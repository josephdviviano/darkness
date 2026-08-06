$input v_texcoord0

// Pyramid downsample: exact 2x box reduction, one level of the bloom
// progressive-downsample chain (VIS-3b).
//
// WHY A PYRAMID EXISTS AT ALL
//
// The 5-tap kernel in fs_bloom_blur.sc is a 9-tap Gaussian collapsed onto
// bilinear pairs, and it is only a Gaussian at its designed tap spacing
// (0, +-1.3846, +-3.2308 texels). Widening the step does not widen the blur,
// it turns the kernel into a sparse comb with unsampled gaps — that aliases,
// and it reads on screen as grain. So a wide radius cannot come from a bigger
// step, and reaching NewDark's documented bloom_range by iteration alone
// needs roughly ten passes.
//
// Halving the resolution instead doubles the radius in screen terms while
// leaving the step at one texel, where the kernel is valid. That is what this
// pass is for: radius comes from pyramid DEPTH, not from step size.
//
// WHY FOUR TAPS AND NOT ONE
//
// For an exact 2x reduction a single bilinear fetch at the destination texel
// centre is already an exact 2x2 average — the centre lands precisely on the
// boundary between two source texels in both axes, so the hardware's 50/50
// weighting is the box filter. That holds only while the source is exactly
// twice the destination, and it stops holding as soon as a level has an odd
// dimension (180 -> 90 -> 45 -> 22 -> 11 is three odd steps out of four).
// Once the ratio drifts off 2.0 the single tap slides off the boundary,
// weights the two texels unevenly, and the level starts shimmering under
// camera motion.
//
// Four taps at the four source-texel CENTRES are immune to that: they are
// point samples of the exact texels being averaged, so the result is the box
// filter whether or not the ratio came out even. This is the same
// construction and the same reasoning as fs_bloom_extract.sc's 4x box, with
// the offset sized for a 2x reduction instead.

#include <bgfx_shader.sh>

SAMPLER2D(s_texBlurSrc, 0);

// u_blurStep.xy = SOURCE texel size (1/srcWidth, 1/srcHeight)
uniform vec4 u_blurStep;

void main()
{
	// Half a source texel from the destination centre is a source texel
	// centre, because the destination centre sits on the boundary.
	vec2 o = u_blurStep.xy * 0.5;

	vec3 color  = texture2D(s_texBlurSrc, v_texcoord0 + vec2(-o.x, -o.y)).rgb;
	color += texture2D(s_texBlurSrc, v_texcoord0 + vec2( o.x, -o.y)).rgb;
	color += texture2D(s_texBlurSrc, v_texcoord0 + vec2(-o.x,  o.y)).rgb;
	color += texture2D(s_texBlurSrc, v_texcoord0 + vec2( o.x,  o.y)).rgb;

	gl_FragColor = vec4(color * 0.25, 1.0);
}
