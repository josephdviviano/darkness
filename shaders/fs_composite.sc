$input v_texcoord0

// Composite pass — adds bloom, then resolves the HDR scene target down to
// the display backbuffer.
//
// The colour-correction stages reproduce NewDark's software
// colour-correction shader exactly — same formulas AND same order — from
// new_dark/doc/sw_cc/cc.fx in the public NewDark 1.28 distribution
// (SatGammaPS, lines 19-41):
//
//     saturation + RGB filter  ->  gamma  ->  contrast + brightness
//
// That ordering is worth stating because it is not the obvious one: gamma
// lands in the MIDDLE, and contrast/brightness comes LAST, which means the
// contrast pivot of 0.5 operates in gamma-encoded space rather than on the
// linear signal. Reordering these stages would leave the knobs with the
// same names but different meanings, so a user's NewDark cc values would
// silently not transfer. Do not "tidy" this into a more conventional order.
//
// Exposure and tone mapping have no NewDark counterpart — sw_cc has neither,
// because its input is already an 8-bit display-referred image. Ours are HDR
// prefix stages and necessarily run before the whole cc chain.
//
// BLOOM IS ADDED BEFORE TONE MAPPING — a deliberate divergence from HPL2
// (Amnesia: The Dark Descent, GPLv3, Frictional Games), whose bloom is the
// last thing that touches the image. HPL2 has no tone mapper and an 8-bit
// LDR chain throughout, so it had no other option. We have a float target
// and a curve, and bloom is scene light: it must be exposed and rolled off
// with everything else, or bright glows clip flat while the rest of the
// image is being tone mapped around them.
//
// IDENTITY PROPERTY — load-bearing, do not break:
// with bloom off, exposure 1, tonemap "none", brightness 0, contrast 1,
// saturation 1, filter (1,1,1) and gamma 1, every stage below is
// algebraically the identity and the clamp reproduces exactly the hardware
// clamp the direct-to-backbuffer path already applied. Enabling
// post-processing at default settings must therefore leave the image
// bit-identical to the pre-post-process renderer. That is what makes this
// safe to layer under original missions.

#include <bgfx_shader.sh>

SAMPLER2D(s_texScene, 0);
SAMPLER2D(s_texBloom, 1);
SAMPLER2D(s_texHalation, 2);

// u_ccParams0:   x = exposure, y = tonemap operator, z = gamma, w = unused
// u_ccParams1:   x = brightness, y = contrast, z = saturation, w = unused
// u_ccFilter:    rgb = RGB colour filter, a = unused
// u_lumaWeights: rgb = luminance weights (the saturation pivot), a = unused
// u_bloomParams: rgb = luma weights * bloom intensity (amnesia style),
//                w   = bloom enabled (0/1)
// u_bloomStyle:  x = style (0 = amnesia, 1 = newdark),
//                y = bloomscale, z = bloom saturation  (both newdark-only)
// u_curveParams: operator-specific curve constants, precomputed on the CPU
//                because they involve several pow() of uniforms that would
//                otherwise be paid per pixel. Meaning depends on the operator:
//                  agx    — x = contrast, y = toe_a, z = slope, w = w
//                  lottes — x = a, y = a*d, z = b, w = c
// u_filmParams:  x = film-stage strength (0 = exact no-op), y = shadow
//                saturation, z = highlight saturation, w = tone falloff
// u_filmTintLo:  rgb = shadow tint (multiplicative), a = unused
// u_filmTintHi:  rgb = highlight tint (multiplicative), a = unused
// u_halation:    rgb = halation tint, a = strength (0 = off, and also 0
//                when the halation pass did not run this frame)
uniform vec4 u_ccParams0;
uniform vec4 u_ccParams1;
uniform vec4 u_ccFilter;
uniform vec4 u_lumaWeights;
uniform vec4 u_bloomParams;
uniform vec4 u_bloomStyle;
uniform vec4 u_curveParams;
uniform vec4 u_filmParams;
uniform vec4 u_filmTintLo;
uniform vec4 u_filmTintHi;
uniform vec4 u_halation;

vec3 tonemapReinhard(vec3 c)
{
	return c / (1.0 + c);
}

// ACES filmic approximation — Krzysztof Narkowicz's public-domain curve fit
// of the ACES RRT+ODT ("ACES Filmic Tone Mapping Curve", 2015). A fit, not
// the real ACES transform: cheap, and the highlight rolloff is the point.
//
// Kept because it is what shipped, but note what it does to this renderer's
// signal: it lifts middle grey 0.18 to 0.267 and desaturates warm highlights
// hard (a 2.0/1.0/0.4 torch leaves as 0.91/0.80/0.54, nearly yellow-white).
// The two curves below are the modern alternatives — see VIS-3.
vec3 tonemapACES(vec3 c)
{
	float a = 2.51;
	float b = 0.03;
	float d = 2.43;
	float e = 0.59;
	float f = 0.14;
	return clamp((c * (a * c + b)) / (c * (d * c + e) + f), 0.0, 1.0);
}

// ── AgX ───────────────────────────────────────────────────────────────────
//
// Adapted from Godot 4's AgX (godotengine/godot, MIT), which is itself an
// approximation of EaryChow's AgX as used by Blender. Godot's version
// substitutes the "allenwp" analytic curve for Blender's LUT so the whole
// transform is closed-form. See Godot
// drivers/gles3/shaders/tonemap_inc.glsl (tonemap_agx / allenwp_curve) and
// servers/rendering/storage/environment_storage.cpp
// (environment_get_tonemap_parameters), which computes the curve constants
// on the CPU. Curve source: https://allenwp.com/blog/2025/05/29/allenwp-tonemapping-curve/
//
// WHY THIS FITS THIS RENDERER PARTICULARLY WELL
//
// TASKS.TODO.md VIS-3 warns that our signal is roughly 0..2 rather than
// physically-scaled radiance, so a curve tuned for real HDR input would need
// its exposure remapped rather than adopted verbatim. AgX needs no such
// remap, and the reason is a coincidence worth writing down: Godot's SDR
// configuration uses a white point of 2.0 (environment_storage.cpp:238-249,
// `MAX(2.0, env->white)`), and our ceiling is 2.0 because of the lightmap
// overbright convention. Measured on the exact constants below:
//
//     input 0.18 (middle grey)  ->  0.18000   exactly preserved
//     input 2.00 (our ceiling)  ->  1.00000   exactly display white
//
// So the curve's own anchor points already land on our signal's anchor
// points. Do NOT "helpfully" rescale the input to AgX.
//
// Contrast is a uniform now rather than a folded literal, so the remaining
// three constants (toe_a, slope, w) are derived from it on the CPU each frame
// — see computeCurveParams() in PostProcess.h, which is Godot's own CPU-side
// derivation. Lowering contrast LIFTS the shadows and SHORTENS the shoulder;
// raising it deepens and lengthens both. Measured at 1.40 the toe finally
// starts compressing (gradient 0.95 at deep shadow) but the shoulder only
// reaches 0.80 of scene range, which is why the Lottes curve below exists.
#define AGX_CROSSOVER    0.18
#define AGX_SHOULDER_MAX 0.82

vec3 agxCurve(vec3 x)
{
	// Reinhard-like shoulder above middle grey.
	vec3 s = x - vec3_splat(AGX_CROSSOVER);
	vec3 slopeS = vec3_splat(u_curveParams.z) * s;
	s = slopeS * (1.0 + s / u_curveParams.w) / (1.0 + slopeS / AGX_SHOULDER_MAX);
	s += vec3_splat(AGX_CROSSOVER);

	// Sigmoid power toe below it.
	vec3 t = pow(x, vec3_splat(u_curveParams.x));
	t = t / (t + vec3_splat(u_curveParams.y));

	// step() rather than lessThan(): bgfx's shader dialect has no bvec mix
	// across every backend profile shaderc targets here.
	vec3 useShoulder = step(vec3_splat(AGX_CROSSOVER), x);
	return mix(t, s, useShoulder);
}

vec3 tonemapAgX(vec3 c)
{
	// Negative input through the inset matrix reads as darker AND more
	// saturated than intended, so clamp first. Our scene cannot currently
	// produce negatives, but bloom and future negative lights could.
	c = max(c, vec3_splat(0.0));

	// Rec.709 -> Rec.2020 combined with the AgX inset, and the inverse
	// outset combined with Rec.2020 -> Rec.709. Godot folds each pair into
	// one matrix to save a multiply; the values are its own.
	//
	// Written as explicit dot products rather than a mat3. GLSL fills a mat3
	// literal column-by-column while HLSL fills it row-by-row, and bgfx's
	// mtxFromRows/mtxFromCols swap definitions between the two profiles, so a
	// matrix literal here is a silent transpose waiting to happen on one
	// backend. These rows are Godot's column-major literals transposed once,
	// by hand, and they are the same rows tonecurves.py validated against.
	vec3 inset0 = vec3(0.544814746488245,  0.373787398372697,  0.0813978551390581);
	vec3 inset1 = vec3(0.140416948464053,  0.754137554567394,  0.105445496968552);
	vec3 inset2 = vec3(0.0888104196149096, 0.178871756420858,  0.732317823964232);

	vec3 outset0 = vec3( 1.96488741169489,  -0.855988495690215, -0.108898916004672);
	vec3 outset1 = vec3(-0.299313364904742,  1.32639796461980,  -0.0270845997150571);
	vec3 outset2 = vec3(-0.164352742528393, -0.238183969428088,  1.40253671195648);

	c = vec3(dot(inset0, c), dot(inset1, c), dot(inset2, c));
	c = agxCurve(c);

	// Clipping before the outset suppresses a cyan cast on very bright input.
	c = min(c, vec3_splat(1.0));

	// The outset deliberately pushes chroma back out, which can leave a
	// saturated channel above 1.0 (measured max 1.145 over the 0..2 cube) or
	// a hair below 0 (measured min -0.011). Godot leaves both in place
	// because later colour adjustments can use them; the cc chain's own
	// clamp below absorbs them here. Do not clamp again at this point — that
	// would flatten exactly the chroma the outset exists to restore.
	return vec3(dot(outset0, c), dot(outset1, c), dot(outset2, c));
}

// ── Lottes ────────────────────────────────────────────────────────────────
//
// Timothy Lottes' tone curve, from "Advanced Techniques and Optimization of
// HDR Color Pipelines" (GDC 2016). Chosen for one property the others do not
// have: an explicitly parameterised SHOULDER, decoupled from the midtone
// anchor.
//
//     f(x) = x^a / (x^(a*d) * b + c)
//
// `a` is contrast and `d` is shoulder; `b` and `c` fall out of pinning the
// curve through two points, which is the reason this operator is here.
//
// WHY THIS AND NOT AgX
//
// AgX's midtone behaviour is right for this renderer and its highlight
// rolloff is not: measured, AgX spends only 0.76 of scene range traversing
// the top quarter of its rolloff, against 1.36 here — a shoulder roughly
// 1.8x longer, which is what reads as a film highlight rather than a digital
// one. AgX also has essentially no toe (gradient 1.013 at deep shadow, i.e.
// linear); this sits at 0.986, so deep shadow genuinely compresses the way a
// film toe does.
//
// **midIn and midOut are both pinned to 0.18 on the CPU.** That is a
// deliberate configuration, not Lottes' default (his midOut is 0.267, an
// ACES-like midtone lift). Pinning them equal is what gives this curve the
// same "middle grey passes through untouched" property that made AgX the
// right base in the first place — measured 0.18 -> 0.18000 exactly. Do not
// unpin them without re-reading TASKS.TODO.md VIS-3a: a lifted midtone is
// the single thing that makes this game's warm, already-bright albedo read
// as fantasy rather than gloom.
//
// **The clamp is load-bearing.** Unlike AgX this curve is NOT bounded above:
// with d < 1 the numerator outgrows the denominator, so f keeps climbing
// past 1.0 once x exceeds `white`. Measured at the shipped parameters,
// f(6.0) = 1.097 and f(12.0) = 1.156. `white` is calibrated so that
// f(white) == 1.0 exactly; everything beyond it clips, which is correct and
// is what the clamp is for. Bloom and halation can both push above the
// scene's own 2.0 ceiling, so this is reached in practice.
vec3 tonemapLottes(vec3 c)
{
	vec3 x = max(c, vec3_splat(1e-6));
	vec3 xa  = pow(x, vec3_splat(u_curveParams.x));
	vec3 xad = pow(x, vec3_splat(u_curveParams.y));
	return clamp(xa / (xad * u_curveParams.z + u_curveParams.w), 0.0, 1.0);
}

// ── Khronos PBR Neutral ───────────────────────────────────────────────────
//
// Khronos Group's "PBR Neutral" tone mapper (2024), from the published
// reference implementation at github.com/KhronosGroup/ToneMapping
// (Apache-2.0). Designed so that material albedo survives tone mapping
// unshifted, which is why it is neutral rather than filmic: it desaturates
// only at the very top of the range and leaves hue alone below it.
//
// Two behaviours to know before judging it on screen:
//
//  * The black-point offset runs UNCONDITIONALLY, ahead of the compression
//    test, so it is not the identity even at low levels — a neutral grey v
//    below 0.76 comes out at exactly v - 0.04. On Thief's already dark
//    content that reads as slightly deeper blacks, and it is the reference
//    behaviour, not a bug.
//  * It stays inside [0,1] on its own (verified over the whole 0..2 cube),
//    unlike AgX. No clamp is needed here.
vec3 tonemapPBRNeutral(vec3 c)
{
	float startCompression = 0.8 - 0.04;
	float desaturation     = 0.15;

	c = max(c, vec3_splat(0.0));

	float x = min(c.r, min(c.g, c.b));
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	c -= vec3_splat(offset);

	float peak = max(c.r, max(c.g, c.b));
	if (peak < startCompression)
	{
		return c;
	}

	float d = 1.0 - startCompression;
	float newPeak = 1.0 - d * d / (peak + d - startCompression);
	c *= newPeak / peak;

	float g = 1.0 - 1.0 / (desaturation * (peak - newPeak) + 1.0);
	return mix(c, vec3_splat(newPeak), g);
}

// ── Film response ─────────────────────────────────────────────────────────
//
// Two operations that a single global saturation scalar cannot express, run
// after the tone curve in display-referred space.
//
//  1. LUMINANCE-DEPENDENT SATURATION. Desaturates the gloom while leaving
//     bright warm sources alone. This is the load-bearing half: Thief's
//     texture set is 66.8% warm by pixel and 0.412 mean saturation
//     (measured over all 1,700 world textures), so a global desaturation
//     dulls the torches along with everything else, and no desaturation
//     leaves the whole world reading brown.
//
//  2. SPLIT TONE, multiplicative. Multiplicative so it can only ever darken:
//     a tint that could add light would undo the gloom it exists to create,
//     and would break the identity property that "post-processing at
//     defaults is bit-identical to post-processing off".
//
// **THE SHADOW TINT DEFAULTS TO NEUTRAL, AND THAT IS DELIBERATE.** The
// obvious move here is cool shadows against warm highlights, and taken even
// slightly too far it produces literally blue shadows — the Thief 3 look,
// which reads as a filter rather than as darkness. Real gloom is inky and
// achromatic. The engine agrees, for what it is worth: across all 32 retail
// missions of Thief 1 and 2 the authored ambient is never tinted, not once,
// and Thief 1 has no field to tint it with. So the shadow work here is
// DESATURATION, not colouring, and the shadow tint ships at (1,1,1) with the
// knob available for anyone who wants a whisper of it.
//
// The whole stage is lerped against its input by `strength`, so 0 is an
// exact no-op and the effect has one master dial rather than five.
vec3 filmResponse(vec3 color)
{
	float L = dot(color, u_lumaWeights.rgb);
	float t = clamp(L, 0.0, 1.0);
	t = t * t * (3.0 - 2.0 * t);              // smoothstep

	// 1. saturation, ramped from shadow value to highlight value
	float sat = mix(u_filmParams.y, u_filmParams.z, t);
	color = mix(vec3_splat(L), color, sat);

	// 2. split tone. pow() concentrates each tint at its own end of the
	//    range and leaves the midtones clean; higher falloff = more confined.
	float Lc = clamp(L, 0.0, 1.0);
	float ws = pow(1.0 - Lc, u_filmParams.w);
	float wh = pow(Lc,       u_filmParams.w);
	color *= mix(vec3_splat(1.0), u_filmTintLo.rgb, ws);
	color *= mix(vec3_splat(1.0), u_filmTintHi.rgb, wh);

	return color;
}

void main()
{
	vec3 color = texture2D(s_texScene, v_texcoord0).rgb;

	// ── Bloom ──
	// Two constructions, selected by bloom_style. They differ in where the
	// "only bright things glow" decision is made, which is what makes them
	// look different rather than merely differently-tuned.
	vec3 bloom = texture2D(s_texBloom, v_texcoord0).rgb;
	vec3 bloomAdd;

	if (u_bloomStyle.x > 0.5)
	{
		// NewDark: the selection already happened in the bright-pass
		// (fs_bloom_extract.sc). What remains is to desaturate the glow
		// toward its own luminance and scale it — bloom_saturation and
		// bloomscale, defaults 0.7 and 5.
		float bloomLuma = dot(bloom, u_lumaWeights.rgb);
		vec3 desat = mix(vec3_splat(bloomLuma), bloom, u_bloomStyle.z);
		bloomAdd = desat * u_bloomStyle.y;
	}
	else
	{
		// HPL2/Amnesia: no bright pass anywhere. The blurred image is
		// weighted by its own luminance before being added, so response is
		// quadratic in brightness — dim regions contribute almost nothing
		// while bright ones bloom hard. That is what lets it run
		// thresholdless without washing out the frame, and it avoids the
		// hard cutoff a threshold puts on a slowly-brightening surface.
		//
		// It suits an HDR source even better than the LDR one it was
		// written for: overbright pixels (>1.0, which the lightmap 2x
		// convention produces) land above unity on both terms and separate
		// sharply from ordinary lit geometry.
		bloomAdd = bloom * dot(bloom, u_bloomParams.rgb);
	}

	color += bloomAdd * u_bloomParams.w;

	// ── Halation ──
	// On film, light that penetrates the emulsion scatters off the base and
	// reflects back, and because long wavelengths penetrate deepest the halo
	// it leaves is red-orange. It is the artifact that reads most immediately
	// as "photochemical" around a flame or a lamp.
	//
	// Reconstructed, not simulated: the spatial spread is borrowed from the
	// bloom pyramid, which has already done the expensive part, and only the
	// tint and strength are ours. Added here, WITH the bloom and before the
	// tone curve, because halation happens at the negative — it is part of
	// the exposure, not a grade on top of it.
	// Its OWN texture, not the bloom one: hard-thresholded so only genuinely
	// overbright sources contribute, and blurred to a small radius so the
	// halo hugs the source. Tinting the bloom texture instead — which is what
	// this used to do — inherited bloom's width and, in the amnesia style,
	// bloom's total lack of a threshold, so everything glowed slightly red
	// across the whole frame. That is the opposite of the effect.
	vec3 halo = texture2D(s_texHalation, v_texcoord0).rgb;
	color += halo * u_halation.rgb * u_halation.a;

	// Exposure, applied while the signal is still unclamped.
	color *= u_ccParams0.x;

	// Tone map. "none" clamps, which is what the backbuffer did implicitly
	// before this pass existed — see the identity property above.
	//
	// Operator ids are the wire format shared with ToneMapOperator in
	// PostProcess.h and with the config/console enum — keep them stable.
	// Compared against midpoints because the value arrives as a float
	// uniform: 0 none, 1 reinhard, 2 aces, 3 agx, 4 pbrneutral.
	float op = u_ccParams0.y;
	if (op > 4.5)
	{
		color = tonemapLottes(color);
	}
	else if (op > 3.5)
	{
		color = tonemapPBRNeutral(color);
	}
	else if (op > 2.5)
	{
		color = tonemapAgX(color);
	}
	else if (op > 1.5)
	{
		color = tonemapACES(color);
	}
	else if (op > 0.5)
	{
		color = tonemapReinhard(color);
	}
	else
	{
		color = clamp(color, 0.0, 1.0);
	}

	// ── Film response ──
	// Between the curve and the cc chain: the curve decides the tonal
	// response, this decides how colour behaves across it, and the cc chain
	// is the user's own grade on top. Lerped by strength so 0 is exact.
	color = mix(color, filmResponse(color), u_filmParams.x);

	// ── cc.fx chain, in its own order — see the header note ──

	// 1. Saturation, then the RGB filter (sepia and friends live here).
	//    cc.fx: lerp(lumi.xxx, vColor.xyz, g_fSaturation) * g_fColorFilter.xyz
	float luma = dot(color, u_lumaWeights.rgb);
	color = mix(vec3_splat(luma), color, u_ccParams1.z);
	color = color * u_ccFilter.rgb;

	// Clamp before the gamma pow(). cc.fx does not, and does not need to:
	// its input is an 8-bit frame, so the only way out of range is a
	// saturation above 1, which extrapolates the lerp and can go negative.
	// pow(negative, gamma) is NaN — a black or garbage pixel rather than a
	// clipped one.
	//
	// This clamp is also where AgX's deliberate out-of-gamut output lands:
	// its outset matrix leaves saturated channels up to ~1.145 and down to
	// ~-0.011, which is the reference behaviour and not something to fix
	// upstream. Every other operator arrives already in range.
	color = clamp(color, 0.0, 1.0);

	// 2. Gamma — mid-chain, not last. cc.fx: pow(vColor.xyz, g_fGamma)
	color = pow(color, vec3_splat(u_ccParams0.z));

	// 3. Contrast about the 0.5 pivot, then brightness as an additive
	//    offset. This is cc.fx's current formula; NewDark notes the older
	//    T2 v1.25 / SS2 v2.46 form was `color * contrast + brightness`,
	//    which it describes as "not quite correct".
	//    cc.fx: saturate((vColor.xyz - 0.5) * g_fContrast + 0.5 + g_fBrightness)
	color = clamp((color - 0.5) * u_ccParams1.y + 0.5 + u_ccParams1.x, 0.0, 1.0);

	// Grain used to be here. It now runs as its OWN pass AFTER antialiasing
	// (kViewGrain, fs_grain.sc) — applied here it sat before SMAA, so SMAA
	// antialiased it and its strength depended on whether AA was enabled.
	// See NOTES.FILM_GRAIN.md §9.1.

	gl_FragColor = vec4(color, 1.0);
}
