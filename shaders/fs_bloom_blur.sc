$input v_texcoord0

// Separable Gaussian blur for the bloom chain. Run twice per iteration
// (horizontal then vertical) at quarter resolution.
//
// Pipeline shape follows HPL2's bloom from Amnesia: The Dark Descent
// (GPLv3, Frictional Games) — quarter-res blur buffers, two ping-ponged
// programs per iteration, iteration count and blur size as parameters.
// See HPL2/core/sources/graphics/PostEffect_Bloom.cpp (RenderBlur).
//
// HPL2 uses two separate programs for the two directions; we use one
// program with the step vector as a uniform, which is the same work with
// one fewer shader to embed.
//
// Taps are the standard 9-tap Gaussian collapsed to 5 samples by placing
// the off-centre taps between texels and letting hardware bilinear do the
// pairwise weighting. This is why the blur source must be LINEAR-sampled:
// with point sampling the fractional offsets snap to texel centres and the
// kernel silently degenerates into a boxy 5-tap.

#include <bgfx_shader.sh>

SAMPLER2D(s_texBlurSrc, 0);

// u_blurStep.xy = per-tap UV step (texel size * direction * blur size)
uniform vec4 u_blurStep;

void main()
{
	vec2 st = u_blurStep.xy;

	vec3 color  = texture2D(s_texBlurSrc, v_texcoord0).rgb * 0.2270270270;
	color += texture2D(s_texBlurSrc, v_texcoord0 + st * 1.3846153846).rgb * 0.3162162162;
	color += texture2D(s_texBlurSrc, v_texcoord0 - st * 1.3846153846).rgb * 0.3162162162;
	color += texture2D(s_texBlurSrc, v_texcoord0 + st * 3.2307692308).rgb * 0.0702702703;
	color += texture2D(s_texBlurSrc, v_texcoord0 - st * 3.2307692308).rgb * 0.0702702703;

	gl_FragColor = vec4(color, 1.0);
}
