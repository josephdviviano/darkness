// Shared by fs_lightmapped.sc and fs_lightmapped_bicubic.sc — the dominant-
// direction specular term (PLAN.HIGH_RES_SHADOWS.md §S6 "reflections").
//
// The re-bake records, per texel, WHERE its light came from: octahedral
// dominant direction in RG, directionality ratio in B (1 = one light,
// 0 = directions cancel), hemisphere openness in A. This turns that into a
// physically-styled rough specular tuned for MOOD, not gloss:
//
//  * Schlick Fresnel with per-material F0/F90: dielectrics reflect 2–5% at
//    normal incidence and only approach full reflectance at grazing angles,
//    so the sheen lives on contours, silhouettes and receding floors — the
//    "plastic" look is exactly what specular WITHOUT this term produces
//    (filmicworlds.com/blog/everything-has-fresnel). F90 sits below 1 for
//    rough/fibrous materials (microfacet self-shadowing at grazing).
//  * Normalised Blinn-Phong ((n+8)/8): higher exponents CONCENTRATE energy
//    instead of adding it, so tight glints stay small and broad sheens stay
//    dim — no over-brightening.
//  * Metals tint the highlight by albedo; dielectric highlights are white.
//  * Energy comes from the L0 lightmap sample — flicker glints — gated by
//    the directionality ratio (multi-lit areas stay matte) and openness
//    (crevices do not sparkle).
//
// Normals come from screen-space derivatives, EXACT for this engine's flat
// world polygons. GLSL-120-safe throughout. The octahedral decoder must stay
// the exact mirror of octahedralEncode/Decode in LightmapBake.h.
//
// Per-draw uniforms (world draws batch per texture, so material presets come
// from SpecularMaterials.h at zero shader cost):
//   u_specParams  : x = Blinn exponent, y = F0·master, z = F90·master
//                   (z carries the enable: 0 when no direction layer),
//                   w = metalness (albedo tint).
//   u_camPosWorld : xyz = camera position in world space.
//
// CMake mirrors this file's mtime onto both .sc consumers
// (.lm_specular_stamp) because bgfx_compile_shaders() does not track .sh
// includes — edit it without that and the bytecode stays silently stale.

vec3 lmOctaDecode(vec2 e)
{
    e = e * 2.0 - 1.0;
    vec3 v = vec3(e.x, e.y, 1.0 - abs(e.x) - abs(e.y));
    float t = max(-v.z, 0.0);
    v.x += (v.x >= 0.0) ? -t : t;
    v.y += (v.y >= 0.0) ? -t : t;
    return normalize(v);
}

vec3 lmSpecular(vec4 dirTex, vec3 worldPos, vec3 camPos, vec3 lightRgb,
                vec3 albedoRgb, vec4 specParams)
{
    if (specParams.y + specParams.z <= 0.0) return vec3(0.0, 0.0, 0.0);
    vec3 l = lmOctaDecode(dirTex.xy);
    vec3 v = normalize(camPos - worldPos);
    // Face normal from derivatives — exact on flat polygons. Winding depends
    // on screen orientation, so orient toward the viewer explicitly.
    vec3 n = normalize(cross(dFdy(worldPos), dFdx(worldPos)));
    n = n * sign(dot(n, v));
    vec3 h = normalize(l + v);

    // Schlick Fresnel between the material's F0 and its (rough-reduced) F90.
    float base = 1.0 - max(dot(v, h), 0.0);
    float e5 = base * base;
    e5 = e5 * e5 * base;
    float fresnel = specParams.y + (specParams.z - specParams.y) * e5;

    // Normalised Blinn-Phong lobe, gated by directionality and openness.
    float norm = (specParams.x + 8.0) / 8.0;
    float lobe = norm * pow(max(dot(n, h), 0.0), specParams.x)
               * dirTex.z * dirTex.w;

    // Dielectric highlights are white; metals tint by albedo.
    vec3 tint = mix(vec3(1.0, 1.0, 1.0), albedoRgb, specParams.w);
    return lightRgb * tint * (lobe * fresnel);
}
