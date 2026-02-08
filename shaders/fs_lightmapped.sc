$input v_texcoord0, v_texcoord1

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);
SAMPLER2D(s_texLightmap, 1);

void main()
{
    vec4 diffuse = texture2D(s_texColor, v_texcoord0);
    // Alpha test: discard transparent pixels (palette index 0 = alpha 0)
    if (diffuse.a < 0.5) discard;
    vec4 light = texture2D(s_texLightmap, v_texcoord1);
    gl_FragColor = vec4(diffuse.rgb * light.rgb * 2.0, diffuse.a);
}
