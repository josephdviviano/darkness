$input v_color0, v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

void main()
{
    vec4 texColor = texture2D(s_texColor, v_texcoord0);
    vec4 finalColor = texColor * v_color0;
    // Alpha test: discard transparent pixels (palette index 0 = alpha 0)
    if (finalColor.a < 0.5) discard;
    gl_FragColor = finalColor;
}
