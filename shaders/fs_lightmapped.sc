$input v_texcoord0, v_texcoord1, v_fogDist

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);
SAMPLER2D(s_texLightmap, 1);

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;
// x = lightmap scale. See graphics.lightmap_scale — this is the "2X modulate"
// multiplier, and whether it belongs here at all is content-dependent:
// NewDark's docs say 16-bit lightmaps (which is what retail ships) top out at
// "texture seen fullbright", i.e. x1, and that overbrightening is the 32-bit
// 2X mode only. We default to 2.0 because it is the look this renderer was
// tuned against, NOT because retail authored for it. NOTES.PROJECT.md
// "Lightmap scale" has the evidence.
uniform vec4 u_lightmapScale;

void main()
{
    vec4 diffuse = texture2D(s_texColor, v_texcoord0);
    // Alpha test: discard transparent pixels (palette index 0 = alpha 0)
    if (diffuse.a < 0.5) discard;
    vec4 light = texture2D(s_texLightmap, v_texcoord1);
    vec4 finalColor = vec4(diffuse.rgb * light.rgb * u_lightmapScale.x, diffuse.a);
    // Linear distance fog
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = finalColor;
}
