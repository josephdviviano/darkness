$input v_color0, v_texcoord0, v_fogDist

#include <bgfx_shader.sh>

// Shadow-face debug HUD (S1): visualise one light's six R32F face tiles.
// The stored value is reach-normalised linear distance (ShadowFaceMath.h):
// 0 = at the light, 1 = cleared / nothing within reach. Shown as-is in
// grayscale — a face that renders pure white is a face with no occluders,
// which is itself informative. Reuses vs_textured (fog varying ignored).

SAMPLER2D(s_texColor, 0);

void main()
{
    float d = texture2D(s_texColor, v_texcoord0).r;
    gl_FragColor = vec4(vec3_splat(d), 1.0) * v_color0;
}
