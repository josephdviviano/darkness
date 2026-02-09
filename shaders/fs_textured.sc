$input v_color0, v_texcoord0, v_fogDist

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

void main()
{
    vec4 texColor = texture2D(s_texColor, v_texcoord0);
    vec4 finalColor = texColor * v_color0;
    // Alpha test: discard transparent pixels (palette index 0 = alpha 0)
    if (finalColor.a < 0.5) discard;
    // Linear distance fog
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = finalColor;
}
