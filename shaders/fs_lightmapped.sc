$input v_texcoord0, v_texcoord1, v_worldPos, v_fogDist

#include <bgfx_shader.sh>
#include "lm_specular.sh"
#include "live_lights.sh"

SAMPLER2D(s_texColor, 0);
SAMPLER2D(s_texLightmap, 1);
SAMPLER2D(s_texLmDir, 2);

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
// .y = lighting-only debug view (0 = normal, 1 = replace albedo with white).
// Isolating the light field is the only way to actually SEE the lumel grid:
// against a 64x64 stone texture the ~1-world-unit lightmap resolution is
// completely masked by albedo detail. See lightmap_view in the debug console.
uniform vec4 u_lightmapScale;
// Dominant-direction specular (see lm_specular.sh for the layout).
uniform vec4 u_specParams;
uniform vec4 u_camPosWorld;

void main()
{
    vec4 diffuse = texture2D(s_texColor, v_texcoord0);
    // Alpha test: discard transparent pixels (palette index 0 = alpha 0)
    if (diffuse.a < 0.5) discard;
    // Alpha is kept from the real texture so the alpha test above still
    // decides visibility — only the colour is replaced.
    diffuse.rgb = mix(diffuse.rgb, vec3(1.0, 1.0, 1.0), u_lightmapScale.y);
    vec4 light = texture2D(s_texLightmap, v_texcoord1);
    // S4: promoted/live lights add to the sampled lightmap BEFORE
    // the scale, so their brightness path matches the overlays they
    // replace (see live_lights.sh).
    light.rgb += liveLightSum(v_worldPos, u_camPosWorld.xyz);
    vec4 finalColor = vec4(diffuse.rgb * light.rgb * u_lightmapScale.x, diffuse.a);
    // Rough Fresnel specular from the baked dominant light direction —
    // per-material presets arrive via u_specParams (see lm_specular.sh).
    // Zeroed F0/F90 = the identity path (no direction atlas / vintage).
    vec4 dirTex = texture2D(s_texLmDir, v_texcoord1);
    finalColor.rgb += lmSpecular(dirTex, v_worldPos, u_camPosWorld.xyz,
                                 light.rgb, diffuse.rgb, u_specParams);
    // Linear distance fog
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = finalColor;
}
