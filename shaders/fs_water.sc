$input v_color0, v_texcoord0, v_fogDist

// Water fragment shader: texture * vertex color with UV scrolling.
// u_waterParams.x = elapsed time (seconds)
// u_waterParams.y = scroll speed multiplier
// UV offset creates flowing water appearance.

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

uniform vec4 u_waterParams;

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

void main()
{
    // Scroll UVs diagonally over time for a flowing appearance
    float t = u_waterParams.x * u_waterParams.y;
    vec2 uv = v_texcoord0 + vec2(t * 0.3, t * 0.7);
    vec4 texColor = texture2D(s_texColor, uv);
    vec4 finalColor = texColor * v_color0;
    // Linear distance fog (applied before alpha blending with background)
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = finalColor;
}
