$input v_texcoord0

// Pyramid upsample: 3x3 tent filter, blended ADDITIVELY into the next larger
// level (VIS-3b). The return leg of the progressive-downsample chain.
//
// The tent is the blur. Going down the chain each level is a box reduction,
// which on its own would magnify back up as visible blocks; the tent spreads
// each small-level texel across its neighbours as it is added back, and
// because every level is added on the way up the result is a sum of
// progressively wider, progressively dimmer kernels. That sum is a far better
// approximation of a wide Gaussian than any single pass at this cost, and it
// is why the radius can be large without the step ever leaving one texel.
//
// Construction follows the dual-filter / progressive-downsample bloom
// described by Jorge Jimenez in "Next Generation Post Processing in Call of
// Duty: Advanced Warfare" (SIGGRAPH 2014 Advances in Real-Time Rendering
// course), which is the same shape bgfx's own example 38 and Godot's glow use.
//
// The 1/2/1 x 1/2/1 separable tent, written out as 9 taps:
//
//     1 2 1
//     2 4 2   * 1/16
//     1 2 1
//
// Offsets are in SOURCE texels — the small level being read — so the footprint
// is one source texel in each direction and the filter stays scale-correct as
// the chain walks back up.
//
// COMBINE IS A LERP, NOT AN ADD, and that is what keeps the radius knob from
// doubling as a brightness knob.
//
//     dst = scatter * upsampled + (1 - scatter) * dst
//
// Adding instead would put roughly one level's worth of extra energy into the
// image per level, so asking for a wider glow would also ask for a brighter
// one and bloom_range would silently be two settings wearing one name. The
// lerp's weights sum to 1 at every step, so the pyramid REDISTRIBUTES the glow
// across scales rather than accumulating it: width changes, total brightness
// does not. Same reasoning as Unity's "scatter" control.
//
// The blend is the framebuffer's, not this shader's — the destination cannot
// be bound as render target and sampler in the same draw. `scatter` is written
// to OUTPUT ALPHA and the pass runs with BGFX_STATE_BLEND_ALPHA, which makes
// the blend unit evaluate exactly the lerp above. The pass deliberately does
// not write alpha into the target; only the blend unit consumes it.

#include <bgfx_shader.sh>

SAMPLER2D(s_texBlurSrc, 0);

// u_blurStep.xy = SOURCE texel size * tent radius
// u_blurStep.z  = scatter, the lerp weight toward this (wider) level
uniform vec4 u_blurStep;

void main()
{
	vec2 o = u_blurStep.xy;

	vec3 color =
		texture2D(s_texBlurSrc, v_texcoord0 + vec2(-o.x, -o.y)).rgb * 1.0 +
		texture2D(s_texBlurSrc, v_texcoord0 + vec2( 0.0, -o.y)).rgb * 2.0 +
		texture2D(s_texBlurSrc, v_texcoord0 + vec2( o.x, -o.y)).rgb * 1.0 +

		texture2D(s_texBlurSrc, v_texcoord0 + vec2(-o.x,  0.0)).rgb * 2.0 +
		texture2D(s_texBlurSrc, v_texcoord0 + vec2( 0.0,  0.0)).rgb * 4.0 +
		texture2D(s_texBlurSrc, v_texcoord0 + vec2( o.x,  0.0)).rgb * 2.0 +

		texture2D(s_texBlurSrc, v_texcoord0 + vec2(-o.x,  o.y)).rgb * 1.0 +
		texture2D(s_texBlurSrc, v_texcoord0 + vec2( 0.0,  o.y)).rgb * 2.0 +
		texture2D(s_texBlurSrc, v_texcoord0 + vec2( o.x,  o.y)).rgb * 1.0;

	// Alpha carries the scatter weight for the blend unit — see the header.
	gl_FragColor = vec4(color * (1.0 / 16.0), u_blurStep.z);
}
