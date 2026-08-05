$input v_texcoord0

// Bloom source extraction: 4x box downsample, then an optional bright pass.
//
// BOTH styles run this pass, because both need the downsample. Only the
// "newdark" style sets a non-zero threshold; the "amnesia" style passes
// threshold 0 / prescale 1, making the bright-pass stage an exact identity
// and leaving this a pure downsample. HPL2 has no bright pass at all — it
// blurs the scene directly and lets a luminance weighting in the combine
// suppress dim areas (see fs_composite.sc), which is the substantive
// difference between the two looks.
//
// WHY THE BOX FILTER IS FOUR TAPS AND NOT ONE
//
// The destination is quarter resolution, so one output texel covers a 4x4
// block of the source. A single texture2D fetch gets a bilinear 2x2
// average — it never reads 12 of those 16 texels. That undersampling is
// pure aliasing, and it is far more visible here than in a normal
// downsample for two reasons: the threshold immediately downstream turns a
// missed sample into a pixel that pops in and out as the camera moves, and
// bloomscale (NewDark default 5) then multiplies whatever survived. The
// symptom is bloom that looks grainy or sparkly on thin bright features —
// lamp filaments, specular edges, the moon.
//
// Four bilinear taps placed at the centres of the four 2x2 quadrants read
// all 16 texels with equal weight: an exact 4x4 box average. Averaging
// before the threshold also softens the threshold's own hard edge, since
// the value being tested is now an area mean rather than a point sample.
//
// Parameters follow NewDark 1.28's documented semantics
// (new_dark/doc/new_config_vars.txt):
//
//   bloom_threshold  0.0-1.0, default 0.6 — "min value required (for R, G
//                    and B) in order for a pixel to be included in bloom
//                    processing. A high value means only the brightest
//                    surfaces will glow."
//   bloomprescale    default 1 — "bloom intensity multiplier applied before
//                    blurring is applied"
//
// RECONSTRUCTION, not a port: NewDark's bloom shader lives inside
// Thief2.exe as D3D9 bytecode and is not published, so this implements the
// documented parameter meanings rather than the original kernel.
//
// The threshold is applied as a per-channel soft subtract, max(c - t, 0),
// not as a binary include/exclude test. The doc wording ("min value
// required for R, G and B") admits both readings, but a binary gate would
// produce hard-edged blocks of bloom that pop as a surface brightens, which
// is not what NewDark looks like. Soft subtract is also the standard
// construction. Flagged in TASKS.TODO.md as an inference.

#include <bgfx_shader.sh>

SAMPLER2D(s_texBlurSrc, 0);

// u_extractParams: x = threshold, y = prescale, zw = source texel size
uniform vec4 u_extractParams;

void main()
{
	// Quadrant centres sit one source texel from the output texel centre;
	// each bilinear tap then averages its own 2x2, covering the full 4x4.
	vec2 ts = u_extractParams.zw;

	vec3 color  = texture2D(s_texBlurSrc, v_texcoord0 + vec2(-ts.x, -ts.y)).rgb;
	color += texture2D(s_texBlurSrc, v_texcoord0 + vec2( ts.x, -ts.y)).rgb;
	color += texture2D(s_texBlurSrc, v_texcoord0 + vec2(-ts.x,  ts.y)).rgb;
	color += texture2D(s_texBlurSrc, v_texcoord0 + vec2( ts.x,  ts.y)).rgb;
	color *= 0.25;

	// Bright pass. Identity when threshold is 0 and prescale is 1, which is
	// what the amnesia style passes.
	color = max(color - vec3_splat(u_extractParams.x), vec3_splat(0.0));
	color *= u_extractParams.y;

	gl_FragColor = vec4(color, 1.0);
}
