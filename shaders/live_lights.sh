// S4a (PLAN.HIGH_RES_SHADOWS §S4): per-fragment term for LIVE lights —
// lights promoted out of the baked atlas (a door is swinging through
// their cone) plus transient sources like the flashlight test vehicle.
//
// The term must be BRIGHTNESS-CONTINUOUS with the overlay it replaces:
// same half-Lambert response the bake uses (backface culled first, like
// bakeOneLight), same physical falloff 1/(d²+a²) with the light's
// per-light throw intensity K_i PREMULTIPLIED into the colour CPU-side
// (mirrors the object per-vertex path), and the result adds to the
// sampled lightmap BEFORE the lightmap scale so display treatment
// matches the baked path exactly.
//
// The face normal comes from screen-space derivatives — exact on flat
// polygons, the same construction lm_specular.sh ships. Shadows arrive
// in S4b (S1 atlas face lookup); until then a promoted light is
// unshadowed for the frames it is live.
//
// GLSL 120-safe: no integer textures, no dynamic array params — the
// uniforms are declared here and both lightmapped fragment shaders
// share them (edit alongside ObjectIllumination/DoorShadow C++ sides).

#ifndef LIVE_LIGHT_CAP
#define LIVE_LIGHT_CAP 4
#endif

// .x = active light count (uniform flow — safe around derivatives)
uniform vec4 u_liveLightCount;
// .x = emitter a² (shared falloff denominator term)
uniform vec4 u_liveFalloff;
// xyz = world position, .w = reach² (cutoff; contribution there is
// sub-quantisation by construction, so the edge cannot pop)
uniform vec4 u_liveLightPos[LIVE_LIGHT_CAP];
// rgb = bright × brightScale × K_i (throw intensity folded),
// .w = S1 shadow-pool slot (< 0 = unshadowed)
uniform vec4 u_liveLightColor[LIVE_LIGHT_CAP];
// S1 face atlas geometry: .x = tilesPerRow, .y = faceSize/atlasW,
// .z = faceSize/atlasH, .w = 1 when the backend's render-target memory is
// bottom-up (GL) — the CPU readback lookup applies the same flip.
uniform vec4 u_liveShadowInfo;
SAMPLER2D(s_liveShadowAtlas, 3);

// Shadow test against the S1 face atlas — the GLSL MIRROR of
// ShadowFaceMath.h: same face tie-break order (shadowFaceForDirection),
// same basis triples (shadowFaceBasis — pinned by the "face basis
// literals" test), same reach-normalised linear-distance compare.
float liveShadowFactor(vec3 worldPos, vec3 lightPos, float slot,
                       float invReach)
{
    if (slot < 0.0) return 1.0;
    vec3 d = worldPos - lightPos;    // light -> fragment
    vec3 ad = abs(d);
    float face; vec3 fwd; vec3 rgt; vec3 up;
    if (ad.x >= ad.y && ad.x >= ad.z) {
        if (d.x >= 0.0) { face = 0.0; fwd = vec3( 1, 0, 0); rgt = vec3( 0,-1, 0); up = vec3(0, 0, 1); }
        else            { face = 1.0; fwd = vec3(-1, 0, 0); rgt = vec3( 0, 1, 0); up = vec3(0, 0, 1); }
    } else if (ad.y >= ad.z) {
        if (d.y >= 0.0) { face = 2.0; fwd = vec3( 0, 1, 0); rgt = vec3( 1, 0, 0); up = vec3(0, 0, 1); }
        else            { face = 3.0; fwd = vec3( 0,-1, 0); rgt = vec3(-1, 0, 0); up = vec3(0, 0, 1); }
    } else {
        if (d.z >= 0.0) { face = 4.0; fwd = vec3( 0, 0, 1); rgt = vec3(-1, 0, 0); up = vec3(0, 1, 0); }
        else            { face = 5.0; fwd = vec3( 0, 0,-1); rgt = vec3( 1, 0, 0); up = vec3(0, 1, 0); }
    }
    float z = dot(d, fwd);
    if (z <= 1e-4) return 1.0;
    // Clamp a hair inside the tile so filtering can never read a
    // neighbouring light's face.
    float u = clamp(dot(d, rgt) / z * 0.5 + 0.5, 0.002, 0.998);
    float v = clamp(dot(d, up)  / z * 0.5 + 0.5, 0.002, 0.998);
    float tile = slot * 6.0 + face;
    float tpr = u_liveShadowInfo.x;
    float tx = tile - tpr * floor(tile / tpr);   // mod, 120-safe
    float ty = floor(tile / tpr);
    // Clip +y rasterises to the TOP of the tile; texture v=0 is the top
    // row on Metal/D3D and the bottom on GL.
    vec2 uv = vec2((tx + u) * u_liveShadowInfo.y,
                   (ty + (1.0 - v)) * u_liveShadowInfo.z);
    if (u_liveShadowInfo.w > 0.5) uv.y = 1.0 - uv.y;
    float stored = texture2D(s_liveShadowAtlas, uv).r;
    float distNorm = length(d) * invReach;
    // World-space bias against rasterised self-shadow acne; the S1
    // cross-check's sweep machinery is where this number gets re-derived
    // if acne or peter-panning shows.
    float biasNorm = 0.5 * invReach;
    return (distNorm <= stored + biasNorm) ? 1.0 : 0.0;
}

vec3 liveLightSum(vec3 worldPos, vec3 camPos)
{
    vec3 sum = vec3(0.0, 0.0, 0.0);
    int n = int(u_liveLightCount.x);
    if (n <= 0) return sum;
    // Viewer-oriented face normal: a visible fragment faces the camera.
    vec3 nrm = normalize(cross(dFdy(worldPos), dFdx(worldPos)));
    vec3 toCam = camPos - worldPos;
    nrm = nrm * sign(dot(nrm, toCam));
    for (int i = 0; i < LIVE_LIGHT_CAP; ++i) {
        if (i >= n) break;
        vec3 v = u_liveLightPos[i].xyz - worldPos;
        float d2 = dot(v, v);
        if (u_liveLightPos[i].w > 0.0 && d2 > u_liveLightPos[i].w)
            continue;
        float dist = sqrt(max(d2, 1e-6));
        float cosT = dot(nrm, v) / dist;
        // The bake culls backfaces BEFORE half-Lambert — mirror it.
        if (cosT <= 0.0) continue;
        float halfLam = cosT * 0.5 + 0.5;
        float fall = 1.0 / (d2 + u_liveFalloff.x);
        float shadow = liveShadowFactor(
            worldPos, u_liveLightPos[i].xyz, u_liveLightColor[i].w,
            inversesqrt(max(u_liveLightPos[i].w, 1e-6)));
        sum += u_liveLightColor[i].rgb * (halfLam * fall * shadow);
    }
    return sum;
}
