// S4a (PLAN.HIGH_RES_SHADOWS §S4): per-fragment term for LIVE lights —
// lights promoted out of the baked atlas (a door is swinging through
// their cone) plus LIVE EMITTERS — moving light-emitting objects (the
// fan-mission player lantern, fire arrows, carried torches).
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
#define LIVE_LIGHT_CAP 32
#endif

// .x = active light count (uniform flow — safe around derivatives)
uniform vec4 u_liveLightCount;
// .x = emitter a² (shared falloff denominator term)
uniform vec4 u_liveFalloff;
// xyz = world position, .w = reach² (cutoff; contribution there is
// sub-quantisation by construction, so the edge cannot pop)
uniform vec4 u_liveLightPos[LIVE_LIGHT_CAP];
// rgb = bright × brightScale × K_i (throw intensity folded),
// .w = S1 shadow-pool slot (< 0 = unshadowed). S4c DIFFERENTIAL lights
// (a baked light whose door is mid-swing) pack TWO slots:
// .w = 100 + frozenSlot·16 + currentSlot; the term becomes
// shadow(current) − shadow(frozen) — negative where the moving leaf now
// blocks the light, positive where it has swung away, ZERO everywhere
// the two poses agree. The baked overlay stays untouched during the
// swing; this term is the (signed) correction on top of it.
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

// Tile sampler shared by the PCSS path: clamps into the tile so
// filtering can never read a neighbouring light's face.
float liveTileStored(float tx, float ty, float u, float v)
{
    float cu = clamp(u, 0.002, 0.998);
    float cv = clamp(v, 0.002, 0.998);
    vec2 uv = vec2((tx + cu) * u_liveShadowInfo.y,
                   (ty + (1.0 - cv)) * u_liveShadowInfo.z);
    if (u_liveShadowInfo.w > 0.5) uv.y = 1.0 - uv.y;
    return texture2D(s_liveShadowAtlas, uv).r;
}

// PCSS visibility — soft penumbra from the stored LINEAR occluder
// distance: width = emitterA * (dRecv - dOcc) / dOcc, projected to face
// UV (a world offset s at the receiver subtends s/dRecv; u spans the
// 90-degree frustum so uvR = 0.5 * s / dRecv). The faces store linear
// distance precisely so this is computable from one map. Degenerates to
// the 1-tap hard test when emitterA = 0 (the [LUMEL_BAKE] self-test
// path stays bit-exact). ~14 taps: used by the OFFSCREEN lumel bake so
// event re-bakes keep the load bake's soft edges — not by the per-frame
// lightmapped shaders.
float liveShadowFactorPCSS(vec3 worldPos, vec3 lightPos, float slot,
                           float invReach, float emitterA)
{
    if (slot < 0.0) return 1.0;
    vec3 d = worldPos - lightPos;
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
    float u = dot(d, rgt) / z * 0.5 + 0.5;
    float v = dot(d, up)  / z * 0.5 + 0.5;
    float tile = slot * 6.0 + face;
    float tpr = u_liveShadowInfo.x;
    float tx = tile - tpr * floor(tile / tpr);
    float ty = floor(tile / tpr);
    float distNorm = length(d) * invReach;
    float biasNorm = 0.5 * invReach;

    // Blocker search: centre + 4 diagonals over the emitter's projected
    // extent at this depth.
    float searchR = clamp(0.5 * emitterA / max(z, 0.5), 0.004, 0.04);
    float occSum = 0.0;
    float occN = 0.0;
    float s0 = liveTileStored(tx, ty, u, v);
    if (s0 + biasNorm < distNorm) { occSum += s0; occN += 1.0; }
    float s1 = liveTileStored(tx, ty, u - searchR, v - searchR);
    if (s1 + biasNorm < distNorm) { occSum += s1; occN += 1.0; }
    float s2 = liveTileStored(tx, ty, u + searchR, v - searchR);
    if (s2 + biasNorm < distNorm) { occSum += s2; occN += 1.0; }
    float s3 = liveTileStored(tx, ty, u - searchR, v + searchR);
    if (s3 + biasNorm < distNorm) { occSum += s3; occN += 1.0; }
    float s4 = liveTileStored(tx, ty, u + searchR, v + searchR);
    if (s4 + biasNorm < distNorm) { occSum += s4; occN += 1.0; }
    if (occN < 0.5) return 1.0;                       // fully lit
    float dOccN = occSum / occN;                      // reach-normalised
    float ratio = max(distNorm - dOccN, 0.0) / max(dOccN, 1e-4);
    float uvR = clamp(0.5 * emitterA * ratio * invReach / distNorm,
                      0.0, 0.05);
    if (uvR < 0.002)   // sub-texel penumbra: the hard test IS the answer
        return (distNorm <= s0 + biasNorm) ? 1.0 : 0.0;

    // PCF: centre + 8-tap disc at the penumbra radius.
    float lit = (distNorm <= s0 + biasNorm) ? 1.0 : 0.0;
    float t;
    t = liveTileStored(tx, ty, u - 0.7071 * uvR, v);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u + 0.7071 * uvR, v);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u, v - 0.7071 * uvR);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u, v + 0.7071 * uvR);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u - 0.35 * uvR, v - 0.35 * uvR);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u + 0.35 * uvR, v - 0.35 * uvR);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u - 0.35 * uvR, v + 0.35 * uvR);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    t = liveTileStored(tx, ty, u + 0.35 * uvR, v + 0.35 * uvR);
    lit += (distNorm <= t + biasNorm) ? 1.0 : 0.0;
    return lit / 9.0;
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
        float sw = u_liveLightColor[i].w;
        float invReach = inversesqrt(max(u_liveLightPos[i].w, 1e-6));
        float shadow;
        if (sw >= 99.5) {
            // Differential (S4c): current-pose minus frozen-pose.
            float t = sw - 100.0;
            float sFrozen = floor(t / 16.0);
            float sCur = t - sFrozen * 16.0;
            shadow = liveShadowFactor(worldPos, u_liveLightPos[i].xyz,
                                      sCur, invReach)
                   - liveShadowFactor(worldPos, u_liveLightPos[i].xyz,
                                      sFrozen, invReach);
        } else {
            shadow = liveShadowFactor(worldPos, u_liveLightPos[i].xyz,
                                      sw, invReach);
        }
        sum += u_liveLightColor[i].rgb * (halfLam * fall * shadow);
    }
    return sum;
}
