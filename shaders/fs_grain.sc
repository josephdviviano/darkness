$input v_texcoord0

// Film grain — the LAST pass before UI.
//
// Split out of fs_composite for a structural reason, not a stylistic one:
// report §9.1 (NOTES.FILM_GRAIN.md) requires grain to run after antialiasing
// and after any temporal upscale, because AA treats grain as detail to
// smooth away and a temporal upscaler treats it as invalid history. It used
// to live inside the composite, which put it BEFORE SMAA — so SMAA was
// antialiasing our grain, and grain strength silently depended on whether
// antialiasing was switched on.
//
// ── The model ──
//
// This follows the AV1 / AFGS1 architecture (Norkin & Birkbeck, DCC 2018;
// open, royalty-free, publicly specified): a spatially-correlated noise field
// plus a scaling function of local brightness. The scaling function is where
// the physics goes. AV1 stores its field as a template and cuts blocks from
// it; we generate ours per pixel instead, for the reason documented above
// grainHash — a stored template is periodic, and a periodic field is visible.
//
// Chosen over the analytic Zhang et al. formulation deliberately — that work
// is patent-pending (US 2024/0312091 A1) and this project is GPLv3. AV1's
// architecture predates its priority date by five years and carries an
// explicit royalty-free grant.
//
// ── Why the amplitude curve is a PRODUCT ──
//
// Two different physical mechanisms, kept as two factors so neither gets
// quietly folded into the other:
//
//   sqrt(V(u))          emulsion granularity. Derived from the filtered
//                       Boolean model of silver-halide coverage (Newson,
//                       Delon & Galerne 2017). Vanishes at BOTH ends —
//                       at pure black no grains developed, at pure white all
//                       of them did, and neither state can fluctuate. Peaks
//                       at u = 0.436.
//
//   pow(1 - u, push)    process emphasis. PUSHED film — underexposed and
//                       over-developed — carries markedly more grain in the
//                       shadows. That is a property of the development, not
//                       of the emulsion's coverage statistics, and it is the
//                       look this project is after.
//
// The first version of this shader had only the second factor, which made it
// LOUDEST AT PURE BLACK — the single commonest tell of synthetic grain, since
// there is nothing there to modulate. The first factor is what fixes that.
//
// Note the mirror-image error is also available and also wrong: adopting only
// sqrt(V(u)) gives a normally-processed stock with grain peaking in the
// midtones, which is not this game's look. Neither factor alone is right.
//
// sqrt(V(u)) has no closed form — it is an integral — so this is a fitted
// approximation, max error 0.000235 (0.103% of peak, and 17x below an 8-bit
// quantisation step). Endpoints are exact zeros.
// Re-derive with `python3 tools/gen_grain_lut.py`, which prints these
// three #defines and checks the model's boundary conditions.
#define GRAIN_V_A 0.499268
#define GRAIN_V_Q 0.675719
#define GRAIN_V_B 0.049413

#include <bgfx_shader.sh>

SAMPLER2D(s_texScene, 0);

// u_grain:      x = strength, y = grain size in reference-1080p pixels,
//               z = push exponent, w = per-frame seed
// u_grainRes:   x = resolution scale (output height / 1080),
//               y = artistic gain, applied outside the physical amplitude
uniform vec4 u_grain;
uniform vec4 u_grainRes;

// ── Why this is generated per pixel and NOT sampled from a baked tile ──
//
// The first version of this pass fetched a 256x256 pre-blurred template with
// repeat addressing and a per-frame UV offset. That has an unfixable flaw:
// a tile smaller than the screen REPEATS across it — measured autocorrelation
// 1.0000 at a lag of exactly one tile period, which at the shipped settings
// put five identical copies across a 720p frame — and offsetting per frame
// merely TRANSLATES that repeating lattice. A periodic texture sliding around
// is exactly what "a moving pattern" looks like.
//
// Enlarging the tile only postpones it and two octaves only halve it (one
// octave still repeats exactly, so correlation at the period drops to 0.54,
// not 0). Generating the field procedurally removes the concept of a period
// altogether: measured autocorrelation at 256 px is 0.009 and at 512 px
// 0.012, against 1.0000 for the tile.
//
// Hoskins' hash33 (public domain) returns THREE decorrelated values per
// lattice cell for roughly the cost of one hash — measured inter-channel
// correlation <= 0.014 — which is what makes per-channel RGB grain affordable
// here. Per-channel is the physically correct construction: each emulsion
// layer is separately sensitised and grains independently.
vec3 grainHash(vec2 cell, float seed)
{
	vec3 p3 = fract(vec3(cell.x, cell.y, seed) * vec3(0.1031, 0.1030, 0.0973));
	p3 += vec3_splat(dot(p3, vec3(p3.y, p3.x, p3.z) + vec3_splat(33.33)));
	vec3 r;
	r.x = fract((p3.x + p3.y) * p3.z);
	r.y = fract((p3.x + p3.x) * p3.y);
	r.z = fract((p3.y + p3.x) * p3.x);
	// Uniform [0,1) -> zero mean, unit variance. 1/0.2887 = 3.4641.
	return (r - vec3_splat(0.5)) * 3.4641;
}

// White noise filtered by a kernel h has correlation (h*h)/sum(h^2). For a
// Gaussian h of standard deviation s that is exactly exp(-d^2 / 4 s^2) — the
// form the filtered Boolean model predicts (NOTES.FILM_GRAIN.md §4.2). So a
// Gaussian-weighted neighbourhood of hashed white noise gives the right
// correlation SHAPE by construction, where the previous rotated-value-noise
// construction only approximated it (deviation 0.18-0.22).
//
// ── Why 3x3 and not 5x5 ──
//
// The hashes are the cost of this pass: measured, grain was 0.70 ms of a
// 2.30 ms GPU frame at 720p, i.e. ~30% of all GPU work, and it is 9 hashes
// here against 25 before.
//
// The window size is NOT a quality/speed trade. Two constraints fix it, and
// they pull opposite ways:
//
//   TRUNCATION  the kernel is cut off at the window edge R (in cells), so
//               sigma must be small relative to R or the tails are lost.
//   LATTICE     sigma must be LARGE enough to smear neighbouring lattice
//               sites into each other, or the field decays into blobs
//               sitting on lattice points — visible square structure.
//
// The previous 5x5 used sigma = 1.0 cell, which is too wide for a +/-2
// window: measured, it discarded 11.04% of the kernel's mass in the worst
// sub-cell phase — the WORST truncation of any scheme tested, including the
// cheaper ones. The extra 16 taps were not buying fidelity.
//
// Narrowing sigma to 0.55 cells and taking the lattice coarser by the same
// factor keeps the correlation length in pixels EXACTLY `size` (sigma_px =
// sigma_cells * spacing = 0.55 * size/0.55), so nothing user-facing changes,
// and it fits a 3x3 with 4.50% truncation. Measured against the model at
// size 1.5, worst deviation over lags (1,0),(1,1),(2,0),(2,2),(3,0),(3,3):
// 0.0054 here versus 0.0204 for the old 25-tap window. Power spectra of both
// are a single smooth isotropic blob; 2x2 was also tried and is NOT viable —
// it shows the axis-aligned lattice cross plainly (16.6% truncation).
//
// THE WINDOW IS CENTRED ON THE PIXEL, NOT THE CELL. `floor` + dx in [-2,2]
// covers [-2-frac, 2-frac], so at frac=0.5 it holds a tap at -2.5 and omits
// its mirror at +2.5 — an effective kernel that changes with sub-cell phase.
// Rounding instead of flooring puts the residual in [-0.5, 0.5] and makes the
// window symmetric for every phase.
//
// THE SUB-CELL PHASE IS LOAD-BEARING. An earlier version weighted by the
// integer offsets alone, which evaluates the field only at cell centres and
// holds it flat across each cell — a nearest-neighbour upsample. The fraction
// of adjacent pixels with bit-identical value was exactly 1 - 1/size: 33% at
// size 1.5, 88% at size 8. It hid because `size` is scaled by
// outputHeight/1080, so at 720p the default 1.5 becomes exactly 1.0 — the ONE
// configuration where the lattice lands on the pixel grid and the error
// vanishes, and that is where the original measurements were taken.
#define GRAIN_SIGMA_CELLS 0.55
#define GRAIN_INV_2SS     1.652893   // 1 / (2 * 0.55^2)

vec3 grainField(vec2 p, float size, float seed)
{
	// Lattice spacing is size/sigma px, so the correlation length in pixels
	// stays `size` and the config key keeps its meaning.
	vec2 sp   = p * (GRAIN_SIGMA_CELLS / max(size, 0.25));
	vec2 base = floor(sp + vec2_splat(0.5));   // round: centre on the pixel
	vec2 frac = sp - base;                     // phase, in [-0.5, 0.5]

	// The 2D Gaussian is separable: exp(-(a^2+b^2)/2s^2) factors into
	// exp(-a^2/2s^2) * exp(-b^2/2s^2). That turns 25 scalar exp() calls into
	// two vector ones, and — because sum over the grid of (wx_i*wy_j)^2
	// factors the same way — turns the normaliser into two dot products
	// instead of a running accumulation.
	vec3 offs = vec3(-1.0, 0.0, 1.0);
	vec3 ddx  = offs - vec3_splat(frac.x);
	vec3 ddy  = offs - vec3_splat(frac.y);
	vec3 wx   = exp(-ddx * ddx * vec3_splat(GRAIN_INV_2SS));
	vec3 wy   = exp(-ddy * ddy * vec3_splat(GRAIN_INV_2SS));

	// Unrolled deliberately: indexing a vec3 by a loop counter needs the
	// loop to be unrolled by the compiler to stay a constant expression, and
	// the GLSL 1.20 profile bgfx targets gives no guarantee it will be.
	vec3 acc = vec3_splat(0.0);
	acc += grainHash(base + vec2(-1.0, -1.0), seed) * (wx.x * wy.x);
	acc += grainHash(base + vec2( 0.0, -1.0), seed) * (wx.y * wy.x);
	acc += grainHash(base + vec2( 1.0, -1.0), seed) * (wx.z * wy.x);
	acc += grainHash(base + vec2(-1.0,  0.0), seed) * (wx.x * wy.y);
	acc += grainHash(base + vec2( 0.0,  0.0), seed) * (wx.y * wy.y);
	acc += grainHash(base + vec2( 1.0,  0.0), seed) * (wx.z * wy.y);
	acc += grainHash(base + vec2(-1.0,  1.0), seed) * (wx.x * wy.z);
	acc += grainHash(base + vec2( 0.0,  1.0), seed) * (wx.y * wy.z);
	acc += grainHash(base + vec2( 1.0,  1.0), seed) * (wx.z * wy.z);

	// Normalise by sqrt(sum w^2) so the field stays at unit variance and
	// `strength` keeps one meaning whatever the size and phase are.
	return acc * inversesqrt(dot(wx, wx) * dot(wy, wy));
}

void main()
{
	vec3 color = texture2D(s_texScene, v_texcoord0).rgb;

	if (u_grain.x > 0.0)
	{
		// Grain has a fixed physical size on the negative, so it must occupy
		// a fixed FRACTION of the frame however finely that frame is drawn
		// (report §9.4). `size` is quoted against a 1080p reference and
		// scaled by output height, so the look holds from 720p to 4K.
		float size = max(u_grain.y * u_grainRes.x, 0.25);
		vec3 n = grainField(gl_FragCoord.xy, size, u_grain.w);

		// PER CHANNEL, not per luma. Each emulsion layer is separately
		// sensitised, so its grain amplitude follows ITS OWN density — report
		// §7.1 wants three independent instances, and §5.2 says explicitly not
		// to inherit AV1's luma-drives-chroma coupling ("you are not a
		// decoder").
		//
		// Driving all three channels from luma was measurably wrong, not just
		// impure: on saturated colours it applies near-peak amplitude to
		// channels that are at zero. Measured on pure blue at strength 0.1,
		// 50.3% of red and green pixels clipped and the mean lifted by 1.25
		// 8-bit steps — grain in pure black, which is the exact tell this
		// curve exists to remove. It also made grain depend on `luma_mode`,
		// a bloom/saturation setting with no business here.
		vec3 uc = clamp(color, vec3_splat(0.0), vec3_splat(1.0));

		// Guarded so the base is never exactly 0: pow(0, 0) is undefined, and
		// push is allowed to reach 0 (a normally-processed stock).
		vec3 omu = max(vec3_splat(1.0) - uc, vec3_splat(1e-6));

		// Emulsion granularity: zero at both ends, peak at 0.436.
		vec3 sv = GRAIN_V_A * sqrt(uc) * pow(omu, vec3_splat(GRAIN_V_Q))
		        * (vec3_splat(1.0) + GRAIN_V_B * uc);

		// Process emphasis. push = 0 gives a normally-processed stock.
		vec3 push = pow(omu, vec3_splat(u_grain.z));

		// u_grainRes.y is the artistic gain — deliberately OUTSIDE the
		// physical amplitude, so stylisation cannot be mistaken for the model
		// (report §4.4).
		color = clamp(color + n * sv * push * (u_grain.x * u_grainRes.y),
		              0.0, 1.0);
	}

	gl_FragColor = vec4(color, 1.0);
}
