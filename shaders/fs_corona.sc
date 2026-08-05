$input v_color0, v_texcoord0, v_fogDist, v_viewZ

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);
SAMPLER2D(s_texDepth, 1);

// u_fogColor.rgb = fog colour, u_fogParams.x = enabled (0/1), .y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

float coronaSceneDepth(vec2 uv)
{
    return texture2D(s_texDepth, uv).x;
}

#include "corona_depth_fade.sh"

void main()
{
    // Screen UV for the depth lookup. gl_FragCoord and the depth attachment
    // share this view's pixel grid and orientation — the corona view renders
    // at the same size into the same colour texture the world pass used — so
    // no origin flip is needed on any backend.
    vec2 screenUv = gl_FragCoord.xy * u_viewTexel.xy;

    gl_FragColor = coronaShade(texture2D(s_texColor, v_texcoord0), v_color0,
                               v_viewZ, v_fogDist, u_fogParams, screenUv);
}
