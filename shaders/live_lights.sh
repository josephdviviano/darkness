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
// xyz = cone axis * (inner*0.5+0.5), w = outer cosine;
// zero xyz = omni. See liveSpotFactor.
uniform vec4 u_liveLightSpot[LIVE_LIGHT_CAP];
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

// Distance from the light to where a tap's ray meets the receiver plane,
// reach-normalised. Exact on planar lumels, which every WR polygon is.
// `planeD` = dot(n, P - L). Returns `fallback` when the ray is near-parallel
// to the plane (dot(n,dir) ~ 0), where the intersection is unstable and no
// answer is better than a huge one.
float tapPlaneRef(vec3 n, float planeD, vec3 fwd, vec3 rgt, vec3 up,
                  float tu, float tv, float invReach, float fallback)
{
    vec3 dir = normalize(fwd + (2.0 * tu - 1.0) * rgt
                             + (2.0 * tv - 1.0) * up);
    float nd = dot(n, dir);
    if (abs(nd) < 1e-4) return fallback;
    float t = planeD / nd;
    if (t <= 0.0) return fallback;
    return t * invReach;
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
// `recvNormal` is the RECEIVER's plane normal. Every tap of a PCF/PCSS
// kernel looks up a DIFFERENT direction from the light, so comparing them
// all against the centre fragment's own distance is only right when the
// receiver is perpendicular to the light — on a grazing receiver the
// neighbouring taps land on parts of the SAME plane at a different depth
// and read as blocked. That is the grazing acne the --door-diff-diag
// harness classified as gpuPhantomShadow. MEASURED A/B on door 407, same
// build, only this term toggled (the [hard] leg is identical in both runs,
// which is the internal control): gpuPhantomShadow 341 -> 15, a 96%
// reduction.
//
// The fix is exact rather than a bias hack: WR polygons are planar, so the
// reference distance for a tap is where that tap's ray actually meets the
// receiver plane — t = dot(n, P-L) / dot(n, dir). Pass a zero normal to
// keep the old fragment-distance behaviour.
float liveShadowFactorPCSS(vec3 worldPos, vec3 lightPos, float slot,
                           float invReach, float emitterA, vec3 recvNormal)
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

    // Plane-referenced tap depth. planeD = dot(n, P - L); a tap direction
    // dir hits the plane at t = planeD / dot(n, dir), which IS the distance
    // along that ray. Falls back to the fragment distance when the caller
    // passes no normal, or when the ray is near-parallel to the plane.
    float planeD = dot(recvNormal, worldPos - lightPos);
    bool usePlane = dot(recvNormal, recvNormal) > 0.5;

    // Depth bias, DERIVED not tuned: what it must cover is the OCCLUDER's
    // depth quantisation in the face, and a face texel subtends a fixed
    // ANGLE, not a fixed distance. One texel of a 90-degree frustum at
    // faceSize F spans z * 2/F laterally, so at F=256 that is z/128;
    // allowing a moderate receiver slope (x2-x4) puts the coefficient in
    // 0.016z..0.031z.
    //
    // The two acceptance gates pull opposite ways, so both endpoints are
    // recorded rather than implied (door 407 / [LUMEL_BAKE], measured):
    //   0.020 + 0.012z : door-diff 39 phantom / 38 missed, self-test FAIL 7.57
    //   0.050 + 0.030z : door-diff 18 phantom / 171 missed, self-test PASS 1.98
    // A larger bias makes the GPU more LIT, which helps a mostly-lit
    // self-test rect and hurts occlusion detection.
    //
    // A CONSTANT was wrong at both ends: the original 0.5 world units is
    // thicker than a door leaf and swallowed occluders whole; 0.05 let far
    // surfaces self-shadow.
    float biasNorm = usePlane
        ? (0.05 + 0.030 * length(d)) * invReach
        : 0.5 * invReach;

    // Reference depth for a tap at face-UV (tu,tv). u = dot(d,rgt)/z*0.5+0.5
    // inverts to dot(d,rgt)/z = 2u-1, so the tap's ray direction is
    // fwd + (2tu-1)*rgt + (2tv-1)*up.
    #define LIVE_TAP_REF(tu, tv) ( usePlane                                   \
        ? tapPlaneRef(recvNormal, planeD, fwd, rgt, up, (tu), (tv), invReach, \
                      distNorm)                                              \
        : distNorm )

    // Blocker search: centre + 4 diagonals over the emitter's projected
    // extent at this depth.
    float searchR = clamp(0.5 * emitterA / max(z, 0.5), 0.004, 0.04);
    float occSum = 0.0;
    float occN = 0.0;
    float r0 = LIVE_TAP_REF(u, v);
    float s0 = liveTileStored(tx, ty, u, v);
    if (s0 + biasNorm < r0) { occSum += s0; occN += 1.0; }
    float s1 = liveTileStored(tx, ty, u - searchR, v - searchR);
    if (s1 + biasNorm < LIVE_TAP_REF(u - searchR, v - searchR))
        { occSum += s1; occN += 1.0; }
    float s2 = liveTileStored(tx, ty, u + searchR, v - searchR);
    if (s2 + biasNorm < LIVE_TAP_REF(u + searchR, v - searchR))
        { occSum += s2; occN += 1.0; }
    float s3 = liveTileStored(tx, ty, u - searchR, v + searchR);
    if (s3 + biasNorm < LIVE_TAP_REF(u - searchR, v + searchR))
        { occSum += s3; occN += 1.0; }
    float s4 = liveTileStored(tx, ty, u + searchR, v + searchR);
    if (s4 + biasNorm < LIVE_TAP_REF(u + searchR, v + searchR))
        { occSum += s4; occN += 1.0; }
    if (occN < 0.5) return 1.0;                       // fully lit
    float dOccN = occSum / occN;                      // reach-normalised
    float ratio = max(distNorm - dOccN, 0.0) / max(dOccN, 1e-4);
    float uvR = clamp(0.5 * emitterA * ratio * invReach / distNorm,
                      0.0, 0.05);
    if (uvR < 0.002)   // sub-texel penumbra: the hard test IS the answer
        return (r0 <= s0 + biasNorm) ? 1.0 : 0.0;

    // PCF: centre + 8-tap disc at the penumbra radius. Each tap carries its
    // OWN plane-referenced depth (see LIVE_TAP_REF).
    float lit = (r0 <= s0 + biasNorm) ? 1.0 : 0.0;
    float t; float tu; float tv;
    tu = u - 0.7071 * uvR; tv = v;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u + 0.7071 * uvR; tv = v;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u; tv = v - 0.7071 * uvR;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u; tv = v + 0.7071 * uvR;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u - 0.35 * uvR; tv = v - 0.35 * uvR;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u + 0.35 * uvR; tv = v - 0.35 * uvR;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u - 0.35 * uvR; tv = v + 0.35 * uvR;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    tu = u + 0.35 * uvR; tv = v + 0.35 * uvR;
    t = liveTileStored(tx, ty, tu, tv);
    lit += (LIVE_TAP_REF(tu, tv) <= t + biasNorm) ? 1.0 : 0.0;
    #undef LIVE_TAP_REF
    return lit / 9.0;
}

// The Continuous 5-bit STORAGE transform — the exact math LumelAccumulator
// uses (Continuous case, incl. the C1 toe) and that fs_lumel_bake.sc mirrors
// when it bakes an overlay.
//
// Why the per-frame differential needs it too: an overlay on disk is a
// POST-transform quantity. The differential's job is to cancel or restore
// that stored value, so it must be computed as T(E*Vcur) - T(E*Vfrozen).
// Subtracting E*(Vcur - Vfrozen) instead — a PRE-transform quantity — is
// systematically too large, because T subtracts half a quantisation step and
// then applies the toe. The visible result is exactly symmetric: a newly
// shadowed surface goes darker than the ambient+bounce floor it should stop
// at (max(.,0) clamps to black, not to ambient), and a surface the door
// stops shadowing comes back BRIGHTER than its surroundings. Both reported
// from the game 2026-08-08.
//
// T is nonlinear, so T(a) - T(b) != T(a - b): the substitution IS the bug.
// Worst exactly where this game lives — the toe is most curved at low
// levels, and 82% of lit texels sit between 13 and 20/255.
vec3 lmStoredContribution(vec3 c, float vis, float spot)
{
    float peak = max(c.r, max(c.g, c.b));
    if (peak <= 1e-6 || vis <= 0.0) return vec3(0.0, 0.0, 0.0);
    // ORDER MATTERS and mirrors LumelAccumulator: the half-step comes off
    // FIRST, is tested, and only then does the spot factor scale lux.
    // Folding spot into `c` beforehand shifts where the -0.5 lands and
    // makes the two ends disagree inside the cone.
    float lux = peak * 255.0 * vis - 0.5;
    if (lux <= 0.0) return vec3(0.0, 0.0, 0.0);
    lux = min(lux * spot, 255.0);
    if (lux <= 0.0) return vec3(0.0, 0.0, 0.0);
    vec3 m = c * (255.0 / peak);
    vec3 gv = m * (lux * 31.0 / 65536.0);
    vec3 toe = mix(gv - 0.5, 0.5 * gv * gv,
                   vec3(lessThan(gv, vec3(1.0, 1.0, 1.0))));
    return max(toe, vec3(0.0, 0.0, 0.0)) * (8.0 / 255.0);
}

// Spot cone factor, mirroring bakeOneLight: hard cutoff at the outer cosine,
// linear ramp to the inner. `packed` is u_liveLightSpot[i]: xyz = cone axis
// scaled by (inner*0.5+0.5), w = outer cosine. A zero-length xyz is the OMNI
// sentinel — which is also what every non-spot light stores, so omni lights
// take the identical path they always did.
//
// This was MISSING: promoted spot lights had their differential applied over
// the whole reach SPHERE while their baked overlay only ever existed inside
// the cone, so outside the cone the differential added or removed light that
// was never baked.
float liveSpotFactor(vec4 cone, vec3 toFrag)
{
    float len = length(cone.xyz);
    if (len < 1e-4) return 1.0;                  // omni
    float inner = len * 2.0 - 1.0;
    float outer = cone.w;
    float dotVal = dot(toFrag, cone.xyz / len);
    if (dotVal <= outer) return 0.0;
    if (dotVal < inner) {
        float denom = inner - outer;
        return (denom > 1e-6) ? (dotVal - outer) / denom : 1.0;
    }
    return 1.0;
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
            //
            // MIRROR of LiveLightPack.h — kLiveDiffPackBias (100) and
            // kLiveSlotPackBase (128). The radix MUST cover the whole
            // shadow pool: it was 16, written when the pool held 16 slots,
            // and once the pool grew every promoted light whose current
            // slot landed at 16+ decoded to a different pair, so this
            // differenced two unrelated lights' faces. Invisible at rest
            // (a settled door carries no differential) and destroyed the
            // lightmap for the whole duration of a swing.
            // tests/test_live_light_pack.cpp sweeps the entire legal slot
            // range against these exact constants — edit both ends together.
            float t = sw - 100.0;
            float sFrozen = floor(t / 128.0);
            float sCur = t - sFrozen * 128.0;
            // Differential in STORED space (see lmStoredContribution):
            // the quantity being cancelled is a stored overlay, so both
            // ends go through the transform before subtracting.
            // BOTH ends soft. The frozen reference cancels a baked
            // overlay that was computed with 16 emitter samples, i.e. a
            // SOFT partial value in the penumbra. Sampling it with a hard
            // 1-tap test reports 0 where the bake stored something
            // non-zero, so the differential adds back the FULL amount on
            // top of a partial one — systematically too bright, and only
            // on the addition side, which is exactly the asymmetry seen in
            // game (subtraction correct, closing door leaves a too-bright
            // region). PCSS emitter size rides u_liveFalloff.y, the same
            // knob the offscreen bake uses, so the two agree by
            // construction rather than by tuning.
            float vCur = liveShadowFactorPCSS(
                worldPos, u_liveLightPos[i].xyz, sCur, invReach,
                u_liveFalloff.y, nrm);
            float vFro = liveShadowFactorPCSS(
                worldPos, u_liveLightPos[i].xyz, sFrozen, invReach,
                u_liveFalloff.y, nrm);
            float spot = liveSpotFactor(u_liveLightSpot[i], -v / dist);
            vec3 c = u_liveLightColor[i].rgb * (halfLam * fall);
            sum += lmStoredContribution(c, vCur, spot)
                 - lmStoredContribution(c, vFro, spot);
            continue;
        } else {
            shadow = liveShadowFactor(worldPos, u_liveLightPos[i].xyz,
                                      sw, invReach);
        }
        // Plain live emitters (lantern, flashlight) are NOT in the baked
        // atlas, so they add new light rather than cancelling a stored
        // quantity — they stay linear, deliberately.
        sum += u_liveLightColor[i].rgb
             * (halfLam * fall * shadow
                * liveSpotFactor(u_liveLightSpot[i], -v / dist));
    }
    return sum;
}
