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

// u_ccParams0:   x = exposure, y = tonemap operator, z = gamma, w = unused
// u_ccParams1:   x = brightness, y = contrast, z = saturation, w = unused
// u_ccFilter:    rgb = RGB colour filter, a = unused
// u_lumaWeights: rgb = luminance weights (the saturation pivot), a = unused
// u_bloomParams: rgb = luma weights * bloom intensity (amnesia style),
//                w   = bloom enabled (0/1)
// u_bloomStyle:  x = style (0 = amnesia, 1 = newdark),
//                y = bloomscale, z = bloom saturation  (both newdark-only)
uniform vec4 u_ccParams0;
uniform vec4 u_ccParams1;
uniform vec4 u_ccFilter;
uniform vec4 u_lumaWeights;
uniform vec4 u_bloomParams;
uniform vec4 u_bloomStyle;

vec3 tonemapReinhard(vec3 c)
{
	return c / (1.0 + c);
}

// ACES filmic approximation — Krzysztof Narkowicz's public-domain curve fit
// of the ACES RRT+ODT ("ACES Filmic Tone Mapping Curve", 2015). A fit, not
// the real ACES transform: cheap, and the highlight rolloff is the point.
vec3 tonemapACES(vec3 c)
{
	float a = 2.51;
	float b = 0.03;
	float d = 2.43;
	float e = 0.59;
	float f = 0.14;
	return clamp((c * (a * c + b)) / (c * (d * c + e) + f), 0.0, 1.0);
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

	// Exposure, applied while the signal is still unclamped.
	color *= u_ccParams0.x;

	// Tone map. "none" clamps, which is what the backbuffer did implicitly
	// before this pass existed — see the identity property above.
	float op = u_ccParams0.y;
	if (op > 1.5)
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
	// clipped one. This clamp is a no-op for every saturation <= 1 and only
	// engages where the reference would produce NaN anyway.
	color = clamp(color, 0.0, 1.0);

	// 2. Gamma — mid-chain, not last. cc.fx: pow(vColor.xyz, g_fGamma)
	color = pow(color, vec3_splat(u_ccParams0.z));

	// 3. Contrast about the 0.5 pivot, then brightness as an additive
	//    offset. This is cc.fx's current formula; NewDark notes the older
	//    T2 v1.25 / SS2 v2.46 form was `color * contrast + brightness`,
	//    which it describes as "not quite correct".
	//    cc.fx: saturate((vColor.xyz - 0.5) * g_fContrast + 0.5 + g_fBrightness)
	color = clamp((color - 0.5) * u_ccParams1.y + 0.5 + u_ccParams1.x, 0.0, 1.0);

	gl_FragColor = vec4(color, 1.0);
}
