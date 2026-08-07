$input v_worldPos

#include <bgfx_shader.sh>

// Shadow faces store LINEAR DISTANCE from the light, normalised by its
// reach — not device depth. The shadow test is then a plain distance
// compare with a plain sampler on every profile the project targets
// (GLSL 120 has no cube-shadow samplers), it reads back losslessly for the
// CPU cross-check, and no GL-vs-Metal depth-range convention ever enters
// the comparison. See ShadowFaceMath.h.
//
// u_shadowLightPos: xyz = light position (world), w = 1 / reach.
uniform vec4 u_shadowLightPos;

void main()
{
    float d = length(v_worldPos - u_shadowLightPos.xyz) * u_shadowLightPos.w;
    gl_FragColor = vec4(d, 0.0, 0.0, 1.0);
}
