$input a_position, a_color0, a_texcoord0, a_normal
$output v_color0, v_texcoord0, v_fogDist

#include <bgfx_shader.sh>
#include "object_lighting_constants.h"

// Per-object light array set by the C++ side via ObjectIlluminator::
// buildLightArray(). See shaders/object_lighting_constants.h for the
// layout rationale. Only the first u_objectLightCount.x entries are
// meaningful; the loop is gated below.
//
//   u_objectLightLoc[i]    .xyz = world-space light position
//                          .w   = cos(inner) for spotlight; -1 = omni
//   u_objectLightDir[i]    .xyz = spotlight direction (zero for omni)
//                          .w   = cos(outer) for spotlight
//   u_objectLightBright[i] .rgb = pre-scaled radiant intensity
//                          .w   = radius^2 (0 = no cutoff)
uniform vec4 u_objectLightCount;
uniform vec4 u_objectAmbient;
uniform vec4 u_objectLightLoc[OBJECT_LIGHT_CAP];
uniform vec4 u_objectLightDir[OBJECT_LIGHT_CAP];
uniform vec4 u_objectLightBright[OBJECT_LIGHT_CAP];

// u_objectParams.z carries the current submesh's material illum (0..1):
// per-material self-illumination, additive on top of the Lambertian sum.
// Lamps and other emissive surfaces use this so their casings glow
// regardless of normal direction. The original engine's per-vertex path
// applies the same additive: `out[i] += illum * mld_illum_rgb`.
// (.x = alpha, .y = frob highlight — both consumed in fragment.)
uniform vec4 u_objectParams;

void main()
{
    // Object-space → world-space for the lighting math. Lights are
    // stored in world coordinates by the C++ side. bgfx populates
    // u_model[0] from bgfx::setTransform() per draw call.
    vec3 worldPos = mul(u_model[0], vec4(a_position, 1.0)).xyz;
    // mat3 of the model matrix is sufficient because Dark Engine
    // objects use uniform scale (P$Scale broadcasts to all axes via
    // kPropertyNoInherit). For non-uniform scale this would need
    // transpose(inverse(mat3(u_model))) instead.
    vec3 worldNormal = normalize(mul(u_model[0], vec4(a_normal, 0.0)).xyz);

    vec3 total = u_objectAmbient.rgb;

    int n = int(u_objectLightCount.x);
    if (n > OBJECT_LIGHT_CAP) n = OBJECT_LIGHT_CAP;
    for (int i = 0; i < n; ++i) {
        vec4 locW = u_objectLightLoc[i];
        vec3 v = locW.xyz - worldPos;
        float dist2 = dot(v, v);

        // Radius cutoff (matches the engine's per-light radius field).
        float radius2 = u_objectLightBright[i].w;
        if (radius2 > 0.0 && dist2 > radius2) continue;

        // Lambertian × inverse-linear: lt = cos(theta) / dist.
        //   (N · v) / |v|^2 = (|v| cos(theta)) / |v|^2 = cos(theta) / |v|
        // (|N| = 1 since worldNormal is normalized.) Watch the divisor:
        // dividing by dist instead of dist^2 here gives plain cos(theta)
        // with no distance falloff — every vertex within the radius
        // saturates the 1.0 clamp and the engine looks blown out.
        // Backface (lt <= 0) contributes nothing — Lambertian half-space.
        float dist = sqrt(max(dist2, 1e-12));
        float lt = dot(worldNormal, v) / max(dist2, 1e-12);
        if (lt <= 0.0) continue;

        // Spotlight cone falloff. inner == -1 sentinel ⇒ omni.
        float spotScale = 1.0;
        float inner = locW.w;
        if (inner != -1.0) {
            vec3 d = -v / dist;  // unit vector from light → vertex
            float dotVal = dot(d, u_objectLightDir[i].xyz);
            float outer = u_objectLightDir[i].w;
            if (dotVal >= inner) {
                spotScale = 1.0;
            } else if (dotVal <= outer) {
                continue;
            } else {
                float denom = max(inner - outer, 1e-6);
                spotScale = (dotVal - outer) / denom;
            }
        }

        total += lt * spotScale * u_objectLightBright[i].rgb;
    }

    // Material self-illumination floor — additive, not max(). A lamp
    // casing with illum=0.6 glows at 0.6 even on its dark side, and gets
    // additional Lambertian contribution where lights actually face it.
    total += vec3_splat(u_objectParams.z);

    // Per-channel clamp at 1.0 — matches the engine's CLUT path which
    // saturated at 0.99 before palette lookup. Keeps dim textures from
    // washing out toward white when stacked bright lights sum past 1.0.
    v_color0 = vec4(min(total, vec3_splat(1.0)), 1.0);

    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    v_texcoord0 = a_texcoord0;

    // View-space distance for fog — same as the scalar shaders.
    vec3 viewPos = mul(u_modelView, vec4(a_position, 1.0)).xyz;
    v_fogDist = length(viewPos);
}
