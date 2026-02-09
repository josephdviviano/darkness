$input v_color0, v_texcoord0

// Water fragment shader: texture * vertex color with UV scrolling.
// u_waterParams.x = elapsed time (seconds)
// u_waterParams.y = scroll speed multiplier
// UV offset creates flowing water appearance.

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

uniform vec4 u_waterParams;

void main()
{
    // Scroll UVs diagonally over time for a flowing appearance
    float t = u_waterParams.x * u_waterParams.y;
    vec2 uv = v_texcoord0 + vec2(t * 0.3, t * 0.7);
    vec4 texColor = texture2D(s_texColor, uv);
    gl_FragColor = texColor * v_color0;
}
