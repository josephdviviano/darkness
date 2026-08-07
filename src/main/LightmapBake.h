/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

// CPU reference lightmap bake — stage S0 of PLAN.HIGH_RES_SHADOWS.md.
//
// This recomputes the per-lumel light values that the mission already ships,
// from the mission's own static light table, and exists so the result can be
// diffed against the shipped bytes. It is the correctness gate for every later
// stage: the same formula ends up in a bake shader AND a world shader, so an
// error here would be wrong identically in both and invisible in an A/B
// between them.
//
// It is deliberately slow and simple — one visibility raycast per (lumel,
// light) pair through the same `raycastWorld` the object-lighting path uses.
// The shipping bake will be a GPU pass; this is the thing that pass gets
// checked against.
//
// Nothing here runs in a game session.

#pragma once

#include "DarknessMath.h"
#include "WRChunkParser.h"
#include "RenderParamsParser.h"
#include "RayCaster.h"
#include "LightmapAtlas.h"
// For kSunlightDistance. The constant belongs to the engine's sun model, not to
// either consumer, and is deliberately not duplicated here — see CLAUDE.md
// "Keeping the notes true" on facts stated twice.
#include "ObjectIllumination.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <vector>

namespace Darkness {

// ── Lumel sample geometry ───────────────────────────────────────────────────
//
// The renderer maps world position -> lumel UV (buildLightmappedMesh in
// DarknessRendererExtended.h). The bake needs the inverse, and it is exact
// rather than a solve, because of one identity worth stating outright:
//
//   projU and projV ARE the coefficients of the world offset in the texture
//   axis basis. d = projU*axisU + projV*axisV.
//
// The renderer's non-orthogonal branch computes [projU; projV] = G^-1 [pu; pv]
// where G is the Gram matrix [[|U|^2, U.V], [U.V, |V|^2]] and pu/pv are the raw
// dot products — i.e. exactly the basis-coefficient solve. So inverting the
// parameterisation is a multiply-add, with no matrix work and no special case
// for skewed axes.
//
// Lumel (i, j) has its CENTRE at lightmap-space (i + 0.5, j + 0.5), which is
// what the atlas UVs address once divided by lx/ly.
struct LumelGrid {
    int     lx = 0, ly = 0;
    Vector3 origin{0.0f};      // polygon's texture origin vertex
    Vector3 axisU{0.0f}, axisV{0.0f};
    float   baseU = 0.0f, baseV = 0.0f;   // proj coords of lumel (0,0) centre
    float   stepU = 0.25f, stepV = 0.25f; // lmU = 4*projU, so 1 lumel = 0.25
    Vector3 normal{0.0f};      // plane normal, faces INTO the cell's air
    bool    valid = false;

    Vector3 at(int i, int j) const {
        return origin
             + axisU * (baseU + stepU * static_cast<float>(i))
             + axisV * (baseV + stepV * static_cast<float>(j));
    }
};

// Mirrors the wrap correction in buildLightmappedMesh. Must stay identical —
// it decides which 64-lumel window the polygon's UVs live in, and a mismatch
// slides every sample point by a multiple of 16 world units.
inline float lightmapFindWrap(float x) {
    if (x >= 0) return -64.0f * static_cast<int>(x / 64.0f);
    return -64.0f * (-1 + static_cast<int>(x / 64.0f));
}

inline LumelGrid buildLumelGrid(const WRParsedData &wr,
                                uint32_t cellIdx, int polyIdx,
                                int density = 1) {
    if (density < 1) density = 1;
    LumelGrid g;
    if (cellIdx >= wr.numCells) return g;
    const WRParsedCell &cell = wr.cells[cellIdx];
    if (polyIdx < 0 || polyIdx >= cell.numTextured) return g;
    if (polyIdx >= static_cast<int>(cell.lightInfos.size())) return g;

    const WRLightInfo &li = cell.lightInfos[polyIdx];
    if (li.lx <= 0 || li.ly <= 0) return g;

    const WRPolygonTexturing &tex = cell.texturing[polyIdx];
    const WRPolygon &poly = cell.polygons[polyIdx];
    const std::vector<uint8_t> &idx = cell.polyIndices[polyIdx];
    if (poly.count < 3 || idx.size() < static_cast<size_t>(poly.count))
        return g;
    if (tex.originVertex >= idx.size()) return g;
    if (idx[tex.originVertex] >= cell.vertices.size()) return g;

    g.origin = cell.vertices[idx[tex.originVertex]];
    g.axisU  = tex.axisU;
    g.axisV  = tex.axisV;
    g.lx = static_cast<int>(li.lx) * density;
    g.ly = static_cast<int>(li.ly) * density;

    const float m2u = glm::length2(tex.axisU);
    const float m2v = glm::length2(tex.axisV);
    if (m2u < 1e-9f || m2v < 1e-9f) return g;
    const float dotp = glm::dot(tex.axisU, tex.axisV);

    // Same shift the mesh builder applies.
    const float lshU = (0.5f - li.u) + tex.u / 1024.0f;
    const float lshV = (0.5f - li.v) + tex.v / 1024.0f;

    // Replay the forward map over the polygon's own vertices to recover the
    // wrap window — it depends on the minimum lmU/lmV across the polygon, so
    // it cannot be derived from the lightmap descriptor alone.
    float minLmU = 0.0f, minLmV = 0.0f;
    for (int vi = 0; vi < poly.count; ++vi) {
        const uint8_t vIdx = idx[vi];
        if (vIdx >= cell.vertices.size()) continue;
        const Vector3 d = cell.vertices[vIdx] - g.origin;
        const float pu = glm::dot(tex.axisU, d);
        const float pv = glm::dot(tex.axisV, d);
        float projU, projV;
        if (std::abs(dotp) < 1e-6f) {
            projU = pu / m2u;
            projV = pv / m2v;
        } else {
            const float corr  = 1.0f / (m2u * m2v - dotp * dotp);
            projU = pu * (corr * m2v) - pv * (corr * dotp);
            projV = pv * (corr * m2u) - pu * (corr * dotp);
        }
        const float lmU = 4.0f * projU + lshU;
        const float lmV = 4.0f * projV + lshV;
        if (vi == 0 || lmU < minLmU) minLmU = lmU;
        if (vi == 0 || lmV < minLmV) minLmV = lmV;
    }
    const float wrapU = lightmapFindWrap(minLmU);
    const float wrapV = lightmapFindWrap(minLmV);

    // At density d, sub-lumel i covers original lumel-space [i/d, (i+1)/d) and
    // its centre sits at (i + 0.5)/d. With lmU = 4*projU + lshU that gives
    //   projU(i) = ((i + 0.5)/d - wrapU - lshU) / 4
    // which collapses to the 1:1 case at d = 1.
    const float invD = 1.0f / static_cast<float>(density);
    g.baseU = (0.5f * invD - wrapU - lshU) * 0.25f;
    g.baseV = (0.5f * invD - wrapV - lshV) * 0.25f;
    g.stepU = 0.25f * invD;
    g.stepV = 0.25f * invD;

    // Cell planes face inward (findCameraCell: inside = positive distance), so
    // the plane normal already points into the air the surface is lit from.
    if (poly.plane < cell.planes.size())
        g.normal = cell.planes[poly.plane].normal;

    g.valid = glm::length2(g.normal) > 1e-9f;
    return g;
}

// ── Bake formula ────────────────────────────────────────────────────────────
//
// The recorded original recipe (see PLAN.DYNAMIC_LIGHTS.md "The original bake
// formula") is direct-only, per light:
//
//   cosTerm = dot(lightLoc - P, N);   if < 0 -> unlit
//   len     = |lightLoc - P|;         if radius != 0 && len > radius -> unlit
//   f       = cosTerm/len/2 + 0.5     (half-Lambert)
//   f      *= bright / len            (inverse-linear falloff)
//
// Two things in it are NOT established and are what this harness is for, so
// both are switchable rather than assumed:
//
//  * whether `inner`/`outer` are spotlight cone cosines (which is how
//    ObjectIlluminator reads them) or inner/outer DISTANCES for a radial fade
//    (which is how the recorded recipe reads them). The same 48-byte record
//    serves both readings; only a residual can decide.
//  * the brightness scale, and whether an illumination cutoff was applied.
//
// Do not collapse these to constants before the numbers say so — that is the
// mistake HANDOFF.VISUAL_PIPELINE.md §12 records.
struct BakeFormula {
    // MEASURED 2026-08-06, twice: plain cosine first "beat" half-Lambert on
    // raw error — but only because the bake was globally too bright and plain
    // cosine is the dimmer curve. On the SHAPE residual (error after dividing
    // out each variant's own best-fit scale) half-Lambert wins decisively
    // (MISS6 p50 1.0 vs 2.9), matching the original's construction. The flag
    // stays switchable to re-run the experiment, not because it is open.
    bool  halfLambert      = true;   // the original's (cos/2 + 0.5)
    bool  spotAsCone       = true;   // false = inner/outer are distances
    float brightScale      = 1.0f;   // multiplier on WRStaticLight.bright
    float illumCutoff      = 0.0f;   // contributions <= this are dropped
    bool  quantiseLux      = true;   // truncate each light to integer 1/255
    // Lumels per original lumel, per axis. 1 = shipped resolution.
    int   density          = 1;
    // Area-light penumbra. `emitterRadius` is the light's physical size in
    // world units; `penumbraSamples` 1 reproduces the original's hard binary
    // shadow. See visibilityFraction for why density without this cannot
    // produce a smooth lightmap.
    float emitterRadius    = 0.75f;
    int   penumbraSamples  = 16;
    // Receiver-side supersampling factor. Each output lumel is the box average
    // of ss*ss sub-samples across its own footprint. 1 = point probe (what the
    // original does by default), 2 = 4 taps, 3 = 9.
    int   supersample      = 2;
    // MEASURED 2026-08-06: adding the sun the way `bakeSun` models it makes the
    // parity residual MUCH worse, so it is off by default until the real model
    // is established. On MISS2 (sun on, 550 lights), excluding it moved the
    // median error 12.5 -> 5.7 /255, p95 53.6 -> 17.9, and the best-fit
    // brightness scale 0.54 -> 0.88. That is not a tuning difference; the term
    // as written is wrong.
    //
    // What is NOT yet known is which part is wrong. Three candidates, none
    // tested: the shipped lumels may exclude sunlight entirely; sky access may
    // be per-polygon rather than per-cell (slot 0 appears in far more cells
    // than have real sky access, so the cell list is too coarse a filter); or
    // `sunScaledRgb` may need the same bake-time scale division the table's
    // own `bright` values already carry.
    bool  includeSun       = true;
    // MEASURED 2026-08-06: the shipped lumels DO carry the RENDPARAMS ambient.
    // This defaulted to false on the assumption that the atlas stores light
    // only; the first parity run falsified it. MISS6's ambient of 0.08 is
    // exactly 16/255 once quantised to 5:5:5, and 16/255 was precisely the
    // median AND p95 absolute error — i.e. almost every texel was off by the
    // ambient and nothing else. Adding it drops the median to 4/255, which is
    // half of the 5:5:5 step of 8 and therefore the quantisation noise floor.
    bool  includeAmbient   = true;
    // The original pushes the lumel off its polygon plane by this much before
    // tracing. Ours was 0.05 — twice too far, which biases every sample toward
    // the light by half the offset.
    float surfaceOffset    = 0.025f;
    // Unproven rays are retried with a tiny deterministic jitter of the light
    // position — see resolveVisibility() for why, and why the jitter must stay
    // numerical rather than becoming an area-light radius.
    int   visibilityRetries = 4;
    float retryJitter       = 0.03f;
    // ── Visibility construction (the original's, recovered 2026-08-06) ──
    // The original engine casts TWO rays per (lumel, light):
    //  1. a zero-epsilon "clamp" ray from the polygon centre to the lumel
    //     point; if it collides, the lumel point is REPLACED by the collision
    //     point. That is its answer to off-polygon and out-of-world texels —
    //     they are evaluated at the nearest reachable point, not their true
    //     position. The clamp depends on geometry only, so we cast it once
    //     per lumel rather than once per (lumel, light).
    //  2. the visibility ray FROM THE LIGHT to the (possibly clamped) point,
    //     started in the light's own cell — resolved once per light, like the
    //     original resolves it once at light registration. A light outside
    //     every cell casts nothing.
    // Its raycaster also answers strictly two ways: a ray stopping on the
    // crack between polygons is BLOCKED by design ("if a ray hits the edge of
    // a surface, it is stopped"), and there is no retry. Both flags stay
    // switchable so the pre-2026-08-06 surface→light construction can be
    // re-measured (--trace-from-surface / --no-clamp).
    bool  traceFromLight   = true;
    bool  clampToWorld     = true;
    // ── Final-storage model — the prime suspect for the 0.73–0.85 excess ──
    // The original does not sum lights in float and quantise once. Each
    // light's integer lux is pushed through a per-colour lookup table that
    // scales by 31·m/65536 per channel (m = colour normalised to peak 255)
    // and FLOORS to a 5-bit step, then adds into the stored 5:5:5 lumel,
    // clamping each channel at 31. Three measured consequences: ~half a
    // 5-bit step (4/255) is lost per light per lumel; a light contributing
    // under ~8.3/255 vanishes entirely (what the empirical 15/255 cutoff was
    // standing in for); and full brightness stores 30/31 ≈ 0.941 of scale.
    //   Off        — float sum (the pre-2026-08-06 behaviour)
    //   Exact      — integer-faithful model, for parity measurement
    //   Continuous — same scale and expected truncation loss with no
    //                quantisation: the level-matched form the smooth
    //                re-bake wants (matching shipped LEVELS, not its steps)
    enum class FiveBit { Off, Exact, Continuous };
    FiveBit fivebit        = FiveBit::Exact;
    // ── Falloff naturalisation (off for parity; the in-game bake enables) ──
    // Most lights have radius 0, so their reach is bounded ONLY by the
    // baked per-cell lists (§1f) — and a list boundary is a HARD lighting
    // seam across a polygon edge. `reachExpand` widens each cell's list by
    // the union of its portal-neighbours' lists (N hops), letting the
    // natural falloff fade below quantisation instead of being cut; walls
    // still occlude (every ray is still cast), and the sub-quantisation
    // skip below bounds the extra cost.
    int   reachExpand      = 0;
    // Radius-limited lights hard-cut at their radius in the recorded
    // construction. A smoothstep fade over the last FRACTION of the radius
    // removes the ring; 0 preserves the original's hard edge.
    float softRadiusFrac   = 0.0f;
    // ── Physical falloff (the "root it in physics" model, 2026-08-06) ──
    // The engine's inverse-LINEAR falloff never terminates and so must be
    // truncated — every truncation is a border. Physical mode replaces it
    // with a finite-size inverse-square emitter, 1/(r²+a²), calibrated to
    // match the original's illuminance at `falloffAnchor` — near-field look
    // holds, the falloff shape is physical, and reach ends below
    // quantisation ON ITS OWN. Candidate lights then come from GEOMETRY
    // (reach spheres + per-polygon occlusion probes), not authored cell
    // lists. Off for parity; the in-game bake enables it.
    bool  falloffPhysical  = false;
    float falloffAnchor    = 8.0f;
    // ── Throw-derived intensity (see the helpers above physicalAnchors) ──
    // alpha > 0 anchors each light at alpha × its authored throw (clamped
    // to [falloffAnchor, kThrowAnchorMax]) instead of the single global
    // anchor — recovering per-light intensity from the authored data. In
    // throw mode the authored radius hard cut and soft tail are DISABLED
    // (the radius is spent as intensity; the physical curve tapers below
    // quantisation on its own). 0 = off (parity, and the pre-throw
    // physical construction).
    float throwAlpha       = 0.0f;
    // The per-light anchor table, built by the bake driver via
    // physicalAnchors() when throwAlpha > 0. NON-OWNING; nullptr = global
    // anchor everywhere. Never hashed into the cache key — throwAlpha is
    // the generative parameter.
    const std::vector<float> *perLightAnchor = nullptr;
    // ── S3 door shadows (DoorShadowSystem.h) ──
    // Optional dynamic-occluder test for visibility rays: called only
    // after a ray proves WR-clear, and only for lights flagged in
    // extraOverlayLights (by construction no other light's rays can cross
    // a door inside its reach). Plain fn+ctx so BakeFormula stays
    // trivially copyable; must be thread-safe across bake workers.
    bool (*segmentBlockedFn)(const void *ctx, const Vector3 &a,
                             const Vector3 &b) = nullptr;
    const void *segmentBlockedCtx = nullptr;
    // Static-light indices forced into the overlay set (door-adjacent):
    // excluded from the base, each gets its own overlay — but ONLY on
    // polygons inside a door's shadow cone (segment poly->light crosses a
    // door swept sphere; see doorSpheres). That per-poly gate is what
    // keeps both the bake cost and a door event's re-bake bounded: the
    // first cut gated on candidacy alone and MISS6 baked 18,935 buffers
    // for 121 'adjacent' lights (throw-boosted reach spheres touch some
    // door almost everywhere). NON-OWNING; keyed in the cache via the
    // doorShadows flag, not by content (deterministic from mission +
    // reach).
    const std::vector<uint8_t> *extraOverlayLights = nullptr;
    // Door swept spheres (xyz = center, w = radius) for the shadow-cone
    // gate above. NON-OWNING.
    const std::vector<glm::vec4> *doorSpheres = nullptr;

    bool throwMode() const {
        return falloffPhysical && throwAlpha > 0.0f && perLightAnchor;
    }
    float anchorFor(int lightIdx) const {
        if (perLightAnchor && lightIdx >= 0 &&
            lightIdx < static_cast<int>(perLightAnchor->size()))
            return (*perLightAnchor)[lightIdx];
        return falloffAnchor;
    }
};

// Is the light visible from this surface point?
//
// `raycastWorld` answers three ways, not two: blocked, proven clear, or
// UNPROVEN (RayStatus, and `rayStatusProven`). Measured on the retail campaign,
// 0.9%–7.3% of a bake's rays come back unproven and **every single one is
// `DiscardNoPolygon`** — the traversal crossed an exit plane and found no
// polygon on it containing the exit point, so it stopped.
//
// That status cannot simply be mapped to an answer:
//
//  * Both of its emission sites mean "the ray left the cell and did NOT go
//    through a portal", which for a visibility query reads as blocked...
//  * ...except when the exit point lands on the shared EDGE between a solid
//    polygon and a portal on the same plane, where both point-in-polygon tests
//    fail and the ray might legitimately have continued.
//
// So neither answer is right in general, and the discard itself must not be
// changed — it is load-bearing for the player-physics stair probe, which needs
// borderline exits discarded to avoid phantom cascade steps.
//
// The fix is to stop asking a grazing ray: retry with a jitter far smaller than
// any geometry, which moves the exit point off the edge it landed on. Rays that
// were genuinely blocked stay blocked; rays that were clipping a corner
// resolve. The jitter must stay numerical — growing it into an emitter radius
// would silently turn this into an area-light sample and change the answer for
// a different reason. (S2's penumbra sampling is that, deliberately, and is
// separate.)
enum class VisibilityResult { Clear, Blocked, Unresolved };

// ── Sunlight ────────────────────────────────────────────────────────────────
//
// The sun is not a table entry — slot 0 is scratch on disk — and it is NOT the
// same construction as the object-lighting path's virtual sun. Three things
// distinguish it, and the first version of this file got all three wrong:
//
// 1. **The two length constants cancel.** The original scales its sunlight
//    direction vector to a fixed cast length and divides the brightness by that
//    same length, so the term reduces to `brightness * cos(theta)` with no
//    distance falloff at all. It is a directional light, as it should be.
//    `kSunlightDistance` (125) belongs to the OBJECT path's virtual-sun
//    placement and has nothing to do with lumels.
// 2. **The cosine is not negated.** `RenderParams::sunlightNorm` is already
//    `-normalize(sunlightVector)` — it points TOWARD the sun, which is why
//    `ObjectIlluminator` places its virtual sun by adding it. The original
//    negates because its own vector points the other way. Negating ours too is
//    a double negation, and it lights precisely the surfaces facing away from
//    the sun. That was the bug behind "adding the sun makes parity much worse".
// 3. **Sky access is per-LUMEL, not per-cell.** The original casts a ray along
//    the sun direction and requires it to terminate on a SKY polygon — texture
//    index >= 249, the same BACKHACK_IDX the mesh builder skips. Slot 0's
//    presence in a cell's light list means "this cell has some sky access
//    somewhere", which is far coarser than "this lumel can see the sky", so
//    using it as the filter floods shadowed surfaces with sunlight.
constexpr float kSunlightCastLength = 1000.0f;

// Sky texture index. Anything at or above this is sky (BACKHACK_IDX).
constexpr int32_t kSkyTextureIndex = 249;

// Does this point see the sky along the sun direction?
inline bool pointSeesSky(const WRParsedData &wr, const Vector3 &pos,
                         const RenderParams &rp, int32_t cellHint) {
    RayHit hit;
    int32_t terminal = -1;
    const Vector3 skyward = pos + rp.sunlightNorm * kSunlightCastLength;
    // Sky access needs a HIT whose polygon is a sky face. A miss means the ray
    // left the world without touching anything, which is not sky access.
    if (!raycastWorld(wr, pos, skyward, hit, &terminal, cellHint))
        return false;
    return hit.textureIndex >= kSkyTextureIndex;
}

// Directional sunlight at a surface with the given normal, in the 0..1 units
// the atlas stores. The 1/255 converts the engine's 0..255 lux scale into ours;
// the static table's own `bright` values arrive already in 0..1, which is why
// only the sun needs it.
inline Vector3 bakeSun(const RenderParams &rp, const Vector3 &normal) {
    if (!rp.useSun) return Vector3(0.0f);
    const float d = glm::dot(rp.sunlightNorm, normal);
    if (d <= 0.0f) return Vector3(0.0f);
    return rp.sunRgb * (rp.sunBrightness * d / 255.0f);
}

// ── Dominant-direction lightmap (S6 "reflections") ──────────────────────────
//
// Alongside irradiance, the bake can record WHERE the light came from: a
// luminance-weighted sum of unit vectors toward each contributing light.
// Per texel that yields a dominant direction and a directionality ratio
// |Σw·d|/Σw ∈ [0,1] — 1 for a single light, → 0 where many lights cancel.
// One extra RGBA8 atlas carries it (octahedral direction in RG, ratio in B,
// hemisphere openness in A), which is the compact alternative to full SH L1:
// +1 texture instead of ×4, chosen deliberately because SH memory competes
// with density for the same budget (PLAN.HIGH_RES_SHADOWS.md §S6.2). The
// shader turns it into rough specular — the dominant light glinting off
// surfaces — with the specular ENERGY read from the L0 lightmap, so animated
// overlays make the glint flicker even though the direction is static.
//
// The ratio is computed from DIRECT light only (ambient and bounce would
// dampen it slightly; second-order, deferred) and animated lights are
// accumulated at their maximum — a light's direction does not change with
// its intensity.
struct DirAccum {
    Vector3 sum{0.0f};
    float weight = 0.0f;
};

// Octahedral unit-vector encoding, [0,1]² ← S². The GLSL decoder in
// shaders/lm_specular.sh must stay the exact mirror of this — the Catch2
// round-trip test guards the C++ pair; the shader copy is guarded by eye.
inline void octahedralEncode(const Vector3 &nIn, float &u, float &v) {
    Vector3 n = nIn;
    const float len = std::abs(n.x) + std::abs(n.y) + std::abs(n.z);
    if (len < 1e-9f) { u = v = 0.5f; return; }
    n /= len;
    float ox = n.x, oy = n.y;
    if (n.z < 0.0f) {
        const float ax = std::abs(n.x), ay = std::abs(n.y);
        ox = (1.0f - ay) * (n.x >= 0.0f ? 1.0f : -1.0f);
        oy = (1.0f - ax) * (n.y >= 0.0f ? 1.0f : -1.0f);
    }
    u = ox * 0.5f + 0.5f;
    v = oy * 0.5f + 0.5f;
}

inline Vector3 octahedralDecode(float u, float v) {
    const float ex = u * 2.0f - 1.0f;
    const float ey = v * 2.0f - 1.0f;
    Vector3 n(ex, ey, 1.0f - std::abs(ex) - std::abs(ey));
    const float t = std::max(-n.z, 0.0f);
    n.x += (n.x >= 0.0f) ? -t : t;
    n.y += (n.y >= 0.0f) ? -t : t;
    const float len = glm::length(n);
    return len > 1e-9f ? n / len : Vector3(0, 0, 1);
}

// ── Whole-polygon bake ──────────────────────────────────────────────────────

struct BakeStats {
    uint64_t lumels = 0;
    uint64_t rays   = 0;
    uint64_t rayBlocked = 0;
    uint64_t penumbraTrivial = 0; // quorum agreed — wholly lit or wholly dark
    uint64_t penumbraRefined = 0; // straddled an edge — full sample set cast
    uint64_t rayRetried = 0;      // needed at least one jittered retry
    uint64_t rayUnevaluable = 0;  // still unresolved after every retry
    // A lightmap rectangle BOUNDS its polygon, so some texels have no surface
    // under them. They are still sampled by bilinear filtering at the polygon's
    // edge, and the shipped baker had to invent values for them too — so they
    // are not a fair part of a parity residual, and they are the prime suspect
    // for rays that cannot be resolved. Counted separately for both reasons.
    uint64_t sunTests = 0, sunLit = 0;   // sky-access traces, and how many hit sky
    uint64_t lumelsInside = 0, lumelsOutside = 0;
    uint64_t raysInside = 0, raysOutside = 0;
    uint64_t unresolvedInside = 0, unresolvedOutside = 0;
    // Clamp-to-world pre-ray (BakeFormula::clampToWorld): how many lumel
    // targets were moved to the collision point, how many clamp rays could
    // not be evaluated (target kept as-is), and the ray count itself — kept
    // apart from `rays` so blocked/unresolved percentages stay comparable
    // with pre-clamp baselines.
    uint64_t clampRays = 0, clampMoved = 0, clampUnproven = 0;
    // Penumbra origins that left the light's cell and needed a full cell
    // search (light→surface tracing only).
    uint64_t emitterRecell = 0;
    // Lights whose position resolves to no cell — the original refuses to
    // register these, so with light→surface tracing they cast nothing.
    // Set once per bake by resolveLightCells, not merged.
    uint64_t lightsNoCell = 0;
    // Hemisphere gather (S6 bounce + AO).
    uint64_t gatherRays = 0;       // rays cast by the gather phase
    uint64_t gatherSky = 0;        // rays terminating on a sky face
    uint64_t gatherUnproven = 0;   // budget/degenerate — scored as open
    uint64_t lumelsGathered = 0;
    uint64_t bounceSum255 = 0;     // Σ per-lumel bounce peak ×255 (mean = /lumelsGathered)
    uint64_t aoSumMil = 0;         // Σ per-lumel openness ×1e6
    // Dominant-direction atlas: Σ directionality ratio ×1e6 and texel count.
    uint64_t dirRatioSumMil = 0;
    uint64_t dirTexels = 0;
    // Geometric candidate selection (physical falloff): per-(polygon,light)
    // pairs considered by reach-sphere test, culled by the occlusion probes,
    // and the probe rays themselves.
    uint64_t geomCandidates = 0;
    // S3: door-adjacent overlay buffers baked (extras appended past the
    // animflags bits).
    uint64_t doorOverlays = 0;
    uint64_t geomProbeCulled = 0;
    uint64_t geomProbeRays = 0;
    // atlasTexelForPoint failing to round-trip a lumel's own centre — a
    // broken inverse mapping, which would silently misdirect every gather
    // sample. Checked per polygon, must be zero.
    uint64_t roundtripFail = 0;
    // Per-status tally of first-attempt unproven rays. An aggregate percentage
    // cannot distinguish "the sample point was outside every cell" (a bug in
    // how we place lumels) from "the ray ran out of its 64-cell budget" (a real
    // limit of long rays, since most lights are unbounded). Those need
    // different fixes, so count them apart.
    uint64_t byStatus[16] = {};
};

inline const char *rayStatusName(RayStatus s) {
    switch (s) {
    case RayStatus::Unset:               return "Unset";
    case RayStatus::Hit:                 return "Hit";
    case RayStatus::Clear:               return "Clear";
    case RayStatus::ZeroLength:          return "ZeroLength";
    case RayStatus::NoStartCell:         return "NoStartCell";
    case RayStatus::DegenerateGeom:      return "DegenerateGeom";
    case RayStatus::DiscardNoPolygon:    return "DiscardNoPolygon";
    case RayStatus::InvalidPortalTarget: return "InvalidPortalTarget";
    case RayStatus::CellBudget:          return "CellBudget";
    case RayStatus::InvalidCell:         return "InvalidCell";
    }
    return "?";
}

inline VisibilityResult resolveVisibility(const WRParsedData &wr,
                                          const Vector3 &from,
                                          const Vector3 &to,
                                          int32_t cellHint,
                                          const BakeFormula &f,
                                          int *outRetriesUsed = nullptr,
                                          RayStatus *outFirstStatus = nullptr) {
    RayHit hit;
    int32_t terminal = -1;
    const bool firstBlocked = raycastWorld(wr, from, to, hit, &terminal, cellHint);
    if (outFirstStatus) *outFirstStatus = hit.status;
    if (firstBlocked)
        return VisibilityResult::Blocked;
    if (rayStatusProven(hit.status))
        return VisibilityResult::Clear;

    // Build two axes perpendicular to the ray so the jitter moves the exit
    // point across the plane it is grazing rather than along the ray.
    Vector3 dir = to - from;
    const float len = glm::length(dir);
    if (len < 1e-6f) return VisibilityResult::Clear;
    dir /= len;
    Vector3 up = (std::abs(dir.z) < 0.9f) ? Vector3(0, 0, 1) : Vector3(1, 0, 0);
    const Vector3 ax = glm::normalize(glm::cross(dir, up));
    const Vector3 ay = glm::cross(dir, ax);

    static const float kOffsets[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (int r = 0; r < f.visibilityRetries && r < 4; ++r) {
        const Vector3 nudge = (ax * kOffsets[r][0] + ay * kOffsets[r][1])
                            * f.retryJitter;
        RayHit h2;
        int32_t t2 = -1;
        const bool blocked = raycastWorld(wr, from, to + nudge, h2, &t2, cellHint);
        if (outRetriesUsed) *outRetriesUsed = r + 1;
        if (blocked) return VisibilityResult::Blocked;
        if (rayStatusProven(h2.status)) return VisibilityResult::Clear;
    }
    return VisibilityResult::Unresolved;
}

// ── Light→surface visibility (the original's direction) ────────────────────
//
// The portal traversal is NOT symmetric under swapping the endpoints: the
// backface extension is anchored at the ray ORIGIN, the epsilons are
// directional, and the straddle correction runs at the `from` end. So tracing
// from the light is a different computation from tracing to it, and the
// original traces from the light.
//
// The original's raycaster also answers strictly two ways. A ray that stops
// on the crack between polygons is a collision, full stop — it never runs a
// point-in-solid-polygon test, so there is no "no polygon claimed the exit"
// discard and no retry. Mapping our three-way answer onto that:
//   Hit                → Blocked
//   Clear              → Clear
//   DiscardNoPolygon   → Blocked  (the crack stops the ray by design)
//   NoStartCell        → Blocked  (a start outside the world casts nothing)
//   everything else    → Unresolved — failures with no original analogue
//                        (our 64-cell budget; the original loops unbounded)
// No jitter retry here: the retry exists to undo OUR discard semantics,
// which this path does not share.
inline VisibilityResult resolveVisibilityFromLight(const WRParsedData &wr,
                                                   const Vector3 &emitterPos,
                                                   const Vector3 &surfacePoint,
                                                   int32_t emitterCellHint,
                                                   RayStatus *outStatus = nullptr) {
    if (emitterCellHint < 0) {
        if (outStatus) *outStatus = RayStatus::NoStartCell;
        return VisibilityResult::Blocked;
    }
    RayHit hit;
    int32_t terminal = -1;
    const bool blocked =
        raycastWorld(wr, emitterPos, surfacePoint, hit, &terminal,
                     emitterCellHint);
    if (outStatus) *outStatus = hit.status;
    if (blocked) return VisibilityResult::Blocked;
    if (hit.status == RayStatus::Clear) return VisibilityResult::Clear;
    if (hit.status == RayStatus::DiscardNoPolygon ||
        hit.status == RayStatus::NoStartCell)
        return VisibilityResult::Blocked;
    return VisibilityResult::Unresolved;
}

// Is `p` inside this cell (non-negative distance to every inward-facing
// plane, within eps)? Used to validate a penumbra sample origin against the
// light's resolved cell before trusting it as a traversal start hint.
inline bool pointInCellPlanes(const WRParsedData &wr, int32_t cellIdx,
                              const Vector3 &p, float eps = 0.001f) {
    if (cellIdx < 0 || cellIdx >= static_cast<int32_t>(wr.numCells))
        return false;
    for (const auto &pl : wr.cells[cellIdx].planes)
        if (pl.getDistance(p) < -eps) return false;
    return true;
}

// The original resolves each light's containing cell ONCE at registration and
// refuses to register a light outside the world. The light→surface trace
// needs the same table: the traversal's start-cell hint must name the cell
// containing the LIGHT, and passing the lumel's cell instead starts it in the
// wrong place. Slot 0 (the sun) has no cell. Entries of -1 mean the light
// casts nothing; report a nonzero count loudly — it is a divergence from the
// mission author's intent, not a normal condition.
inline std::vector<int32_t> resolveLightCells(const WRParsedData &wr,
                                              uint64_t *outNoCell = nullptr) {
    std::vector<int32_t> cells(wr.staticLights.size(), -1);
    uint64_t noCell = 0;
    for (size_t i = 1; i < wr.staticLights.size(); ++i) {
        const Vector3 &loc = wr.staticLights[i].loc;
        cells[i] = findCameraCell(wr, loc.x, loc.y, loc.z);
        if (cells[i] < 0) ++noCell;
    }
    if (outNoCell) *outNoCell = noCell;
    return cells;
}

// ── Physical falloff and reach (BakeFormula::falloffPhysical) ───────────────
//
// The engine's falloff is inverse-LINEAR (1/r): physically wrong, and the
// direct cause of every hard light border — a 1/r light never gets dim
// enough to terminate, so its reach must be TRUNCATED (per-cell lists, hard
// radii, thresholds), and every truncation is a seam. The physical model is
// an inverse-square emitter of finite size:
//
//     E(r) ∝ 1 / (r² + a²),   a = emitter radius (the light has extent, so
//                             no singularity at r → 0)
//
// calibrated to deliver the SAME illuminance as the original at one anchor
// distance r₀, so the near-field look holds while the falloff SHAPE is
// physical everywhere:
//
//     distFactor(r) = (r₀² + a²) / (r₀ · (r² + a²))     [= 1/r₀ at r = r₀]
//
// Inverse-square dies fast enough that each light's reach ends BELOW
// QUANTISATION on its own: the sub-quantisation radius R_q where the peak
// possible contribution drops under half an 8-bit count is finite and
// computable, so reach needs no authored lists and no feathering — the
// border ceases to exist rather than being smoothed. The per-cell lists are
// rebuilt from R_q via portal BFS (walls still occlude — every ray is still
// cast); the sub-quantisation ray skip bounds the cost.

// Half an 8-bit count: below this a contribution stores nothing in any
// storage mode. Shared by the ray skip and the physical-reach radius.
constexpr float kSubQuantThreshold = 0.5f / 255.0f;
// Reach cap: at this distance even a brightness-20 light (the campaign's
// brightest class) contributes ~1/255. Bounds the BFS on pathological
// emitters without a visible cutoff.
constexpr float kPhysicalReachCap = 200.0f;

inline float physicalDistFactor(float len, float anchor, float emitterA) {
    const float a2 = emitterA * emitterA;
    return (anchor * anchor + a2) / (anchor * (len * len + a2));
}

// Does segment a->b pass within `radius` of `center`? The S3 door
// shadow-cone gate: a polygon needs a door-light overlay only if its
// light ray can cross the door's swept sphere.
inline bool segmentNearSphere(const Vector3 &a, const Vector3 &b,
                              const Vector3 &center, float radius) {
    const Vector3 d = b - a;
    const float len2 = glm::dot(d, d);
    float t = 0.0f;
    if (len2 > 1e-12f)
        t = std::min(1.0f, std::max(0.0f,
                glm::dot(center - a, d) / len2));
    const Vector3 closest = a + d * t;
    return glm::dot(center - closest, center - closest) <= radius * radius;
}

// ── Throw-derived intensity (per-light anchors) ────────────────────────────
//
// The physical falloff SHAPE is right; what the global anchor got wrong is
// per-light INTENSITY. Under the original's 1/r the designers stated each
// light's intended reach — its THROW — twice: an authored radius on ~20% of
// lights (2..80 units, clearly per-fixture), and the per-cell light lists
// for the rest. A single global anchor pins every light to its original
// brightness at one mission-wide distance (8u), which under inverse-square
// leaves a radius-40 fixture at a FIFTH of its authored brightness at its
// own edge. Light is gameplay (the lightgem reads these lumels), so that
// discrepancy is not cosmetic.
//
// Anchoring each light at a FRACTION of its own throw is exactly
// "modulate intensity by throw": scaling the curve and moving its
// 1/r-equivalence point are the same operation. The authored hard cut is
// NOT kept in throw mode — the radius has been spent as intensity, and the
// physical curve tapers below quantisation on its own (smooth, borderless).

// Upper bound on a throw-derived anchor: keeps s = K(anchor)/K(8) within
// ~4x so the near field cannot blow the 8-bit store out arbitrarily far.
constexpr float kThrowAnchorMax = 32.0f;

// Per-light throw from the authored data: the explicit radius where one
// was authored, else the reach of the per-cell light lists (the original's
// actual reach mechanism) — farthest extent of any cell that lists the
// light. Slot 0 (sun) and unlisted lights stay 0.
inline std::vector<float> authoredThrow(const WRParsedData &wr) {
    std::vector<float> throwR(wr.staticLights.size(), 0.0f);
    for (const auto &cell : wr.cells) {
        const auto &lt = cell.lightIndices;
        const int n = lt.empty() ? 0 : static_cast<int>(lt[0]);
        for (int k = 1; k <= n && k < static_cast<int>(lt.size()); ++k) {
            const uint16_t li = lt[k];
            if (li == 0 || li >= throwR.size()) continue;
            const float d =
                glm::length(cell.center - wr.staticLights[li].loc) +
                cell.radius;
            if (d > throwR[li]) throwR[li] = d;
        }
    }
    for (size_t li = 1; li < wr.staticLights.size(); ++li)
        if (wr.staticLights[li].radius > 0.0f)
            throwR[li] = wr.staticLights[li].radius;
    return throwR;
}

// anchor_i = clamp(sqrt(anchorDefault · 2·alpha·throw_i), anchorDefault,
// kThrowAnchorMax) — the GEOMETRIC MEAN of the near-field floor and the
// (alpha-scaled) throw. Measured reason (MISS2 sweep, 2026-08-07): a linear
// alpha·throw law let big LIST-derived throws (outdoor lamps listing many
// large cells) boost intensity ~4x and lift formerly-cut street shadows by
// +63/255 at the tail. The sqrt law is self-limiting: doubling a light's
// throw grows its anchor by √2, so the huge throws that inflate under the
// list derivation stay tame while the mid-range (the actual darkness
// problem) still recovers. At alpha=0.5 this is exactly √(anchor·throw).
// The floor at the global default means no light ever gets DIMMER than the
// uniform-anchor construction; small candles keep today's look exactly.
inline std::vector<float> physicalAnchors(const WRParsedData &wr,
                                          float alpha, float anchorDefault) {
    std::vector<float> anchors(wr.staticLights.size(), anchorDefault);
    if (alpha <= 0.0f) return anchors;
    // List-derived throws (unbounded lights) carry HALF weight and a
    // tighter cap. Measured (MISS2 A/B, 2026-08-07): an authored radius
    // cuts light that is still bright — full intent, full law — while a
    // list edge is where contribution became negligible; treating list
    // reach as full intent lifted formerly-cut outdoor shadows +63/255 at
    // the tail. Radius-only throws left the tail at +6/255 but lost the
    // dark-end recovery; the asymmetric weighting keeps both.
    constexpr float kListThrowWeight = 0.5f;
    constexpr float kListAnchorMax   = 24.0f;
    const std::vector<float> throwR = authoredThrow(wr);
    for (size_t li = 1; li < anchors.size(); ++li) {
        const bool authored = wr.staticLights[li].radius > 0.0f;
        const float tw = authored ? throwR[li]
                                  : throwR[li] * kListThrowWeight;
        const float cap = authored ? kThrowAnchorMax : kListAnchorMax;
        float a = std::sqrt(anchorDefault * 2.0f * alpha * tw);
        if (a < anchorDefault) a = anchorDefault;
        if (a > cap) a = cap;
        anchors[li] = a;
    }
    return anchors;
}

// The sub-quantisation reach radius of a light under physical falloff: the
// distance at which its peak possible contribution drops below half an
// 8-bit count. Pure geometry — no cell lists, no authored reach. The
// authored radius (where present) remains an upper bound: it is a designer
// CONSTRAINT, not a rendering artefact.
inline float physicalReachRadius(const WRStaticLight &L, float anchor,
                                 float emitterA, float brightScale,
                                 bool clampToAuthored = true) {
    const float peak =
        std::max({L.bright.x, L.bright.y, L.bright.z}) * brightScale;
    if (peak <= 0.0f) return 0.0f;
    const float a2 = emitterA * emitterA;
    const float rq2 = peak * (anchor * anchor + a2)
                          / (anchor * kSubQuantThreshold) - a2;
    if (rq2 <= 0.0f) return 0.0f;
    float rq = std::sqrt(rq2);
    if (rq > kPhysicalReachCap) rq = kPhysicalReachCap;
    // Throw mode passes false: the radius is spent as intensity there and
    // the curve tapers on its own — clamping would re-cut the smooth tail.
    if (clampToAuthored && L.radius > 0.0f && L.radius < rq) rq = L.radius;
    return rq;
}

// Hop-expanded AUTHORED lists — the milder variant used when the original
// 1/r falloff is kept (parity-adjacent looks): unions each cell's list with
// its portal-neighbours' to soften list-truncation seams. No feather: with
// physical falloff available this path exists for A/B, not as the default.
inline std::vector<std::vector<uint16_t>> expandCellLightLists(
    const WRParsedData &wr, int hops) {
    std::vector<std::vector<uint16_t>> out(wr.numCells);
    std::vector<uint8_t> inSet(wr.staticLights.size(), 0);
    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        std::vector<uint16_t> members;
        std::vector<uint32_t> frontier{ci}, nextFrontier;
        std::vector<uint32_t> visited{ci};
        auto absorb = [&](uint32_t cell, bool own) {
            const auto &lt = wr.cells[cell].lightIndices;
            const int cnt = lt.empty() ? 0 : static_cast<int>(lt[0]);
            for (int k = 1; k <= cnt && k < static_cast<int>(lt.size()); ++k) {
                const uint16_t idx = lt[k];
                if (idx == 0 && !own) continue;   // sun stays own-list only
                if (idx < inSet.size() && !inSet[idx]) {
                    inSet[idx] = 1;
                    members.push_back(idx);
                }
            }
        };
        absorb(ci, true);
        for (int h = 0; h < hops; ++h) {
            nextFrontier.clear();
            for (uint32_t fc : frontier) {
                const WRParsedCell &cell = wr.cells[fc];
                const int numSolid = cell.numPolygons - cell.numPortals;
                for (int p = numSolid; p < cell.numPolygons; ++p) {
                    const uint32_t tgt = cell.polygons[p].tgtCell;
                    if (tgt >= wr.numCells) continue;
                    if (std::find(visited.begin(), visited.end(), tgt) !=
                        visited.end())
                        continue;
                    visited.push_back(tgt);
                    nextFrontier.push_back(tgt);
                    absorb(tgt, false);
                }
            }
            frontier.swap(nextFrontier);
        }
        for (uint16_t idx : members) inSet[idx] = 0;   // reset for next cell
        out[ci].reserve(members.size() + 1);
        out[ci].push_back(static_cast<uint16_t>(members.size()));
        out[ci].insert(out[ci].end(), members.begin(), members.end());
    }
    return out;
}

// The set of static-table lights that are ANIMATED, read from the cells' own
// animMap tables. animMap entries index the static light table directly —
// established by the lightmap_overlays shape fit (exact best-fit agreement on
// unambiguous overlays; the exceptions are co-located, shape-degenerate
// lights) — and it is the same number the runtime blend already uses to key
// intensities. animated[i] != 0 means light i has overlays somewhere.
inline std::vector<uint8_t> animatedLightSet(const WRParsedData &wr) {
    std::vector<uint8_t> animated(wr.staticLights.size(), 0);
    for (const auto &cell : wr.cells)
        for (int16_t idx : cell.animMap)
            if (idx > 0 && static_cast<size_t>(idx) < animated.size())
                animated[idx] = 1;
    return animated;
}

// ── Area-light visibility (the penumbra) ────────────────────────────────────
//
// A point light with a binary visibility test produces a STEP at the shadow
// edge. Raising lumel density does not smooth a step, it only relocates it —
// the edge gets sharper, not softer. So resolution alone cannot deliver a
// smooth lightmap, and this is the piece that can.
//
// A real emitter has extent. Sampling it at N points and taking the fraction
// that reach the surface gives a genuine gradient whose width grows with
// occluder distance, exactly as a physical penumbra does. The original engine
// has no such term at all; its editor gained one later (the `quad_lighting`
// family of modes), so this is an enhancement with a precedent rather than an
// invention.
//
// Cost is the reason for the two-tier structure below: a full N-sample sweep
// on every lumel would multiply an already slow bake by N. Almost every lumel
// is wholly lit or wholly shadowed, and only the minority straddling an edge
// needs the full set — so take a cheap agreeing quorum first and escalate only
// on disagreement.
constexpr int kPenumbraQuorum = 5;   // centre + 4 ring samples

// Deterministic points on the unit disc: centre, then a 4-point ring, then a
// sunflower spiral for the escalated samples. Deterministic because a bake that
// changes between runs cannot be diffed against itself.
inline void emitterSampleOffset(int i, float &ox, float &oy) {
    if (i == 0) { ox = oy = 0.0f; return; }
    if (i < kPenumbraQuorum) {
        const float a = 1.5707963f * static_cast<float>(i - 1);
        ox = std::cos(a); oy = std::sin(a);
        return;
    }
    // Sunflower / Vogel spiral — even coverage without a lattice.
    const float k = static_cast<float>(i - kPenumbraQuorum) + 0.5f;
    const float r = std::sqrt(k / 24.0f);
    const float a = k * 2.39996323f;   // golden angle
    ox = r * std::cos(a); oy = r * std::sin(a);
}

// Fraction of the emitter visible from `surfacePoint`, in [0,1].
//
// Direction follows f.traceFromLight: either from the emitter point to the
// surface (the original's construction, started in the LIGHT's cell) or from
// the surface to the emitter point (ours, started in the lumel's cell). With
// penumbra sampling the jittered point is always the EMITTER-side one — which
// in from-light mode is the ray ORIGIN, so each sample origin must be
// re-validated against the light's cell before its hint is trusted.
inline float visibilityFraction(const WRParsedData &wr,
                                const Vector3 &surfacePoint,
                                const Vector3 &lightPos,
                                int32_t surfaceCellHint,
                                int32_t lightCellHint,
                                const BakeFormula &f,
                                BakeStats &stats,
                                bool testDynamicOccluders = false) {
    // One emitter point, either direction. Also owns the stats bookkeeping:
    // rayRetried/byStatus count first-attempt-unproven in surface→light mode,
    // and every non-Hit/non-Clear outcome in light→surface mode (there the
    // interesting split is crack-blocked vs budget-exhausted).
    //
    // Dynamic occluders (S3 door leaves): tested only after a WR-clear
    // ray, and only when the caller flagged this light as door-adjacent —
    // every other light's rays cannot cross a door within reach, so the
    // whole base bake pays nothing for the hook.
    auto doorGate = [&](const Vector3 &emitterPos,
                        VisibilityResult v) -> VisibilityResult {
        if (v == VisibilityResult::Clear && testDynamicOccluders &&
            f.segmentBlockedFn &&
            f.segmentBlockedFn(f.segmentBlockedCtx, emitterPos,
                               surfacePoint))
            return VisibilityResult::Blocked;
        return v;
    };
    auto castOne = [&](const Vector3 &emitterPos,
                       int32_t emitterCell) -> VisibilityResult {
        ++stats.rays;
        if (f.traceFromLight) {
            RayStatus st = RayStatus::Unset;
            const VisibilityResult v = resolveVisibilityFromLight(
                wr, emitterPos, surfacePoint, emitterCell, &st);
            if (st != RayStatus::Hit && st != RayStatus::Clear)
                ++stats.byStatus[static_cast<int>(st)];
            return doorGate(emitterPos, v);
        }
        int retries = 0;
        RayStatus first = RayStatus::Unset;
        const VisibilityResult v = resolveVisibility(
            wr, surfacePoint, emitterPos, surfaceCellHint, f, &retries, &first);
        if (retries > 0 || v == VisibilityResult::Unresolved) {
            ++stats.rayRetried;
            ++stats.byStatus[static_cast<int>(first)];
        }
        return doorGate(emitterPos, v);
    };

    const int samples = std::max(1, f.penumbraSamples);
    if (samples == 1 || f.emitterRadius <= 0.0f) {
        const VisibilityResult v = castOne(lightPos, lightCellHint);
        if (v == VisibilityResult::Unresolved) { ++stats.rayUnevaluable; return 0.0f; }
        if (v == VisibilityResult::Blocked)    { ++stats.rayBlocked;    return 0.0f; }
        return 1.0f;
    }

    // Disc basis perpendicular to the light direction, so the sampled emitter
    // always faces the receiver.
    Vector3 dir = lightPos - surfacePoint;
    const float len = glm::length(dir);
    if (len < 1e-6f) return 1.0f;
    dir /= len;
    const Vector3 up = (std::abs(dir.z) < 0.9f) ? Vector3(0, 0, 1) : Vector3(1, 0, 0);
    const Vector3 ax = glm::normalize(glm::cross(dir, up)) * f.emitterRadius;
    const Vector3 ay = glm::cross(dir, ax / f.emitterRadius) * f.emitterRadius;

    int clear = 0, evaluated = 0;
    auto castSample = [&](int i) {
        float ox, oy;
        emitterSampleOffset(i, ox, oy);
        const Vector3 emitterPos = lightPos + ax * ox + ay * oy;
        // A jittered origin can leave the light's cell (the disc is up to
        // emitterRadius wide; cells can be thinner). Trust the light's cell
        // only after checking, and fall back to a full search — an origin in
        // solid is a part of the emitter embedded in a wall, which blocks.
        int32_t emitterCell = lightCellHint;
        if (f.traceFromLight && i != 0 &&
            !pointInCellPlanes(wr, lightCellHint, emitterPos)) {
            emitterCell = findCameraCell(wr, emitterPos.x, emitterPos.y,
                                         emitterPos.z);
            ++stats.emitterRecell;
        }
        const VisibilityResult v = castOne(emitterPos, emitterCell);
        if (v == VisibilityResult::Unresolved) { ++stats.rayUnevaluable; return; }
        ++evaluated;
        if (v == VisibilityResult::Blocked) ++stats.rayBlocked; else ++clear;
    };

    const int quorum = std::min(samples, kPenumbraQuorum);
    for (int i = 0; i < quorum; ++i) castSample(i);
    // Unanimous quorum -> not on an edge, no refinement can change the answer
    // enough to matter. This is where the cost saving lives.
    if (evaluated > 0 && (clear == 0 || clear == evaluated)) {
        ++stats.penumbraTrivial;
        return evaluated ? static_cast<float>(clear) / evaluated : 0.0f;
    }
    ++stats.penumbraRefined;
    for (int i = quorum; i < samples; ++i) castSample(i);
    return evaluated ? static_cast<float>(clear) / evaluated : 0.0f;
}


// Contribution of one static-light slot at a surface point. Visibility is the
// caller's job — it is the expensive half and the part a GPU bake replaces.
// Storage (quantisation, colour tables, cutoff) is LumelAccumulator's job:
// the original checks its cutoff and truncates BEFORE the spotlight factor,
// then multiplies the spot cone into the already-integer value — so the
// pre-spot scalar and the spot factor have to survive to the accumulator
// separately for the truncation order to be reproducible.
struct LightSample {
    Vector3 contribution{0.0f};  // full contribution, spot factor included
    float   preSpotPeak = 0.0f;  // peak channel before the spot factor
    float   spotFactor  = 1.0f;
};

inline LightSample bakeOneLight(const WRStaticLight &L,
                                const Vector3 &pos,
                                const Vector3 &normal,
                                const BakeFormula &f,
                                int lightIdx = -1) {
    LightSample out;
    const Vector3 toLight = L.loc - pos;
    const float cosTerm = glm::dot(toLight, normal);
    if (cosTerm < 0.0f) return out;

    // Throw mode: the authored radius has been SPENT as intensity (see
    // physicalAnchors) — no hard cut, no soft tail; the physical curve
    // tapers below quantisation on its own.
    const bool throwMode = f.throwMode();

    const float len = glm::length(toLight);
    if (len < 1e-6f) {
        out.contribution = L.bright * f.brightScale;
        out.preSpotPeak = std::max({out.contribution.x, out.contribution.y,
                                    out.contribution.z});
        return out;
    }
    if (!throwMode && L.radius > 0.0f && len > L.radius) return out;

    float scale = f.halfLambert ? (cosTerm / len * 0.5f + 0.5f)
                                : (cosTerm / len);
    if (scale <= 0.0f) return out;

    // Soft tail on radius-limited lights (falloff naturalisation): the
    // recorded construction cuts to zero AT the radius, which reads as a
    // hard ring at re-bake resolution. Smoothstep over the last fraction.
    if (!throwMode && f.softRadiusFrac > 0.0f && L.radius > 0.0f) {
        const float fadeStart = L.radius * (1.0f - f.softRadiusFrac);
        if (len > fadeStart) {
            float t = (L.radius - len) / (L.radius - fadeStart);
            if (t < 0.0f) t = 0.0f;
            scale *= t * t * (3.0f - 2.0f * t);
            if (scale <= 0.0f) return out;
        }
    }

    float spot = 1.0f;
    if (f.spotAsCone) {
        // inner == -1 is the omni sentinel; otherwise linear ramp between the
        // two cone cosines, matching ObjectIlluminator::applyOneLight.
        if (L.inner != -1.0f) {
            const Vector3 d = -toLight / len;
            const float dotVal = glm::dot(d, L.dir);
            if (dotVal <= L.outer) return out;
            if (dotVal < L.inner) {
                const float denom = L.inner - L.outer;
                spot = (denom > 1e-6f) ? (dotVal - L.outer) / denom : 0.0f;
            }
        }
    } else {
        // inner/outer as distances: full brightness inside `inner`, linear
        // ramp to zero at `radius`.
        if (L.inner > 0.0f && L.radius > L.inner && len > L.inner)
            scale *= (L.radius - len) / (L.radius - L.inner);
    }
    if (spot <= 0.0f) return out;

    // Distance response: the recorded engine's inverse-linear, or the
    // physical finite-emitter inverse-square (see BakeFormula). The anchor
    // is per-light under throw mode — that IS the intensity recovery.
    const float distFactor = f.falloffPhysical
        ? physicalDistFactor(len, f.anchorFor(lightIdx), f.emitterRadius)
        : (1.0f / len);
    const Vector3 preSpot = L.bright * (f.brightScale * scale * distFactor);
    out.preSpotPeak = std::max({preSpot.x, preSpot.y, preSpot.z});
    out.spotFactor  = spot;
    out.contribution = preSpot * spot;
    return out;
}

// ── Final-storage accumulator ───────────────────────────────────────────────
//
// Models where each light's contribution LANDS — see BakeFormula::fivebit.
// The original's order per light: float result → cutoff check → truncate to
// integer lux → multiply the spotlight factor into the INTEGER → clamp 255 →
// per-channel colour table (scaled 31·m/65536, floored to a 5-bit step) →
// add into the 5:5:5 lumel, clamping each channel at 31.
//
// Colour is applied to the scalar through the table rather than per-channel
// float math, so the per-channel value here is reconstructed from the
// contribution's channel ratios — the peak channel is the scalar.
struct LumelAccumulator {
    Vector3 linear{0.0f};       // Off / Continuous modes
    int     acc5[3] = {0, 0, 0}; // Exact mode

    // Ambient seeds the accumulator before any light. The exact mechanism of
    // the original's ambient store is unobserved (its value is measured in
    // the shipped data: MISS6's 0.08 stores as 5-bit 2 = 16/255); pushing it
    // through the same white-colour table reproduces that measurement.
    void seedAmbient(const Vector3 &ambient, const BakeFormula &f) {
        switch (f.fivebit) {
        case BakeFormula::FiveBit::Off:
            linear += ambient;
            break;
        case BakeFormula::FiveBit::Exact:
            for (int c = 0; c < 3; ++c) {
                int aLux = static_cast<int>(ambient[c] * 255.0f);
                aLux = std::min(255, std::max(0, aLux));
                acc5[c] = std::min(31, (aLux * 31 * 255) >> 16);
            }
            break;
        case BakeFormula::FiveBit::Continuous:
            for (int c = 0; c < 3; ++c) {
                const float aLux = std::max(0.0f, ambient[c] * 255.0f - 0.5f);
                const float g = aLux * 31.0f * 255.0f / 65536.0f;
                linear[c] += 8.0f * std::max(0.0f, g - 0.5f) / 255.0f;
            }
            break;
        }
    }

    void add(const LightSample &s, float visFrac, const BakeFormula &f) {
        if (s.preSpotPeak <= 0.0f || visFrac <= 0.0f) return;
        switch (f.fivebit) {
        case BakeFormula::FiveBit::Off: {
            // The pre-2026-08-06 behaviour, preserved for A/B: float sum,
            // optional integer-lux truncation of the magnitude with the hue
            // scaled along, cutoff on the post-spot peak.
            Vector3 c = s.contribution * visFrac;
            const float peak = std::max({c.x, c.y, c.z});
            if (peak <= f.illumCutoff) return;
            if (f.quantiseLux && peak > 0.0f) {
                const float lux = std::floor(peak * 255.0f);
                if (lux < 1.0f) return;
                c *= (lux / 255.0f) / peak;
            }
            linear += c;
            break;
        }
        case BakeFormula::FiveBit::Exact: {
            // Cutoff on the PRE-spot scalar, as the original checks it.
            if (s.preSpotPeak <= f.illumCutoff) return;
            int lux = static_cast<int>(s.preSpotPeak * visFrac * 255.0f);
            if (lux <= 0) return;
            if (s.spotFactor < 1.0f)
                lux = static_cast<int>(static_cast<float>(lux) * s.spotFactor);
            if (lux <= 0) return;
            if (lux > 255) lux = 255;
            const Vector3 &c = s.contribution;   // ratios; spot scalar cancels
            const float peak = std::max({c.x, c.y, c.z});
            if (peak <= 0.0f) return;
            for (int ch = 0; ch < 3; ++ch) {
                const int m = static_cast<int>(
                    std::lround(255.0f * c[ch] / peak));
                const int add5 = (lux * 31 * m) >> 16;
                acc5[ch] = std::min(31, acc5[ch] + add5);
            }
            break;
        }
        case BakeFormula::FiveBit::Continuous: {
            // Expected value of Exact with the floors replaced by their mean
            // loss (−0.5 step, clamped at zero): same brightness, no steps.
            //
            // The clamp needs a SMOOTH TOE (formula v2): the raw hinge
            // max(0, g−0.5) kills a light's contribution at exactly half a
            // 5-bit step, and at re-bake resolution that zero-crossing reads
            // as a hard contour ring at the edge of every light's reach —
            // one of the "harsh falloff" borders reported on screen. The C1
            // replacement (g²/2 below g=1, g−0.5 above) keeps the same
            // asymptotic loss while fading to true zero smoothly. Level
            // impact ≤ +½ step inside a 1.5-step band — invisible.
            if (s.preSpotPeak <= f.illumCutoff) return;
            float lux = s.preSpotPeak * visFrac * 255.0f - 0.5f;
            if (lux <= 0.0f) return;
            lux *= s.spotFactor;
            if (lux > 255.0f) lux = 255.0f;
            const Vector3 &c = s.contribution;
            const float peak = std::max({c.x, c.y, c.z});
            if (peak <= 0.0f) return;
            for (int ch = 0; ch < 3; ++ch) {
                const float m = 255.0f * c[ch] / peak;
                const float g = lux * 31.0f * m / 65536.0f;
                const float toe = (g < 1.0f) ? (0.5f * g * g)
                                             : (g - 0.5f);
                linear[ch] += 8.0f * toe / 255.0f;
            }
            break;
        }
        }
    }

    Vector3 finalize(const BakeFormula &f) const {
        if (f.fivebit == BakeFormula::FiveBit::Exact)
            return Vector3(acc5[0] * 8.0f / 255.0f,
                           acc5[1] * 8.0f / 255.0f,
                           acc5[2] * 8.0f / 255.0f);
        return linear;
    }
};

// Bakes one polygon's lumels with EVERY light in its cell's list contributing
// at full brightness. That is deliberately the "all animated lights on" state,
// because it is what the shipped static layer plus all of its overlays sums to
// — and matching that needs no AnimLight property mapping, so this runs without
// the service stack.
//
// `lightCells` is resolveLightCells()' table — required because the
// light→surface trace must start in the LIGHT's cell, and resolving it per
// lumel would repeat a whole-mission cell search millions of times.
//
// `lightMask`, when non-null, restricts which static-table lights contribute
// (mask[i] != 0 → include light i). The sun and ambient are governed by
// f.includeSun / f.includeAmbient as always. This is what lets one driver
// bake "the static base" (everything but the animated set) and "one animated
// light's overlay" (exactly one bit set) with identical machinery — the
// decomposition the shipped data itself uses.
//
// `out` is lx*ly RGB triples, unclamped.
//
// `dirAccum`, when non-null (sized lx*ly at OUTPUT resolution), receives the
// luminance-weighted direction sums ADDED to whatever it already holds — so
// one buffer accumulates across the base call and every overlay call for the
// same polygon, which is how animated lights join the direction field at
// their maximum.
inline bool bakePolygon(const WRParsedData &wr,
                        const RenderParams &rp,
                        uint32_t cellIdx, int polyIdx,
                        const BakeFormula &f,
                        const std::vector<int32_t> &lightCells,
                        std::vector<Vector3> &out,
                        BakeStats &stats,
                        std::vector<uint8_t> *outInsidePolygon = nullptr,
                        const uint8_t *lightMask = nullptr,
                        std::vector<DirAccum> *dirAccum = nullptr,
                        const std::vector<uint16_t> *lightListOverride
                            = nullptr) {
    // RECEIVER-side supersampling. The emitter sampling above softens a shadow
    // EDGE; this is the other half, and it is the one that removes the lumel
    // GRID: each output lumel becomes the average over its own footprint
    // instead of a single point probe, so the cell stops being a flat tile with
    // a hard step to its neighbour.
    //
    // Implemented as "bake at density*ss, box-filter down to density" rather
    // than as N jittered probes per lumel. Same ray budget, but every sample
    // sits at a distinct grid position (no correlation between neighbours), and
    // the reduction is an exact box filter over the footprint. The original
    // engine's equivalent takes 4 taps a full lumel out and averages them,
    // which blurs more than it antialiases; this is the footprint-correct form.
    const int ss = std::max(1, f.supersample);
    const LumelGrid g = buildLumelGrid(wr, cellIdx, polyIdx, f.density * ss);
    if (!g.valid) return false;
    const int outW = g.lx / ss, outH = g.ly / ss;
    if (outW <= 0 || outH <= 0) return false;

    const WRParsedCell &cell = wr.cells[cellIdx];
    const std::vector<uint16_t> &lt =
        lightListOverride ? *lightListOverride : cell.lightIndices;
    const int n = lt.empty() ? 0 : static_cast<int>(lt[0]);

    // Full-resolution scratch; reduced into `out` at the end.
    std::vector<Vector3> fine(static_cast<size_t>(g.lx) * g.ly, Vector3(0.0f));
    std::vector<uint8_t> fineInside;
    if (outInsidePolygon)
        fineInside.assign(static_cast<size_t>(g.lx) * g.ly, 0);
    std::vector<DirAccum> fineDir;
    if (dirAccum)
        fineDir.assign(static_cast<size_t>(g.lx) * g.ly, DirAccum{});

    // Winding normal for the point-in-polygon test. RayCaster uses the NEGATED
    // plane normal for solid faces, and the two must agree or every texel comes
    // back "outside".
    const Vector3 windingNormal = -g.normal;
    const std::vector<uint8_t> &polyIdxList = cell.polyIndices[polyIdx];

    // The clamp ray's anchor: the polygon's own centre, lifted off the plane
    // like every other sample. The on-disk texturing record carries the centre
    // the original engine used.
    const Vector3 centreOff =
        cell.texturing[polyIdx].center + g.normal * f.surfaceOffset;

    for (int j = 0; j < g.ly; ++j) {
        for (int i = 0; i < g.lx; ++i) {
            // Lift the sample off the plane. A lumel sits exactly ON a solid
            // face, where the traversal's own exit-plane search can return the
            // surface itself at t=0 and report a false hit.
            const Vector3 onSurface = g.at(i, j);
            Vector3 p = onSurface + g.normal * f.surfaceOffset;
            int32_t pCell = static_cast<int32_t>(cellIdx);

            const bool inside = pointInConvexPolygon(onSurface, cell.vertices,
                                                     polyIdxList, windingNormal);
            if (inside) ++stats.lumelsInside; else ++stats.lumelsOutside;
            if (outInsidePolygon)
                fineInside[static_cast<size_t>(j) * g.lx + i] = inside ? 1 : 0;

            // Clamp-to-world pre-ray (BakeFormula::clampToWorld): the
            // original casts centre→lumel with EXACT epsilons and, on any
            // collision, evaluates the light AT THE COLLISION POINT instead.
            // Its raycaster treats a portal-less exit plane as a collision
            // without asking which polygon was struck, so our
            // DiscardNoPolygon is a collision here too (the crossing point
            // is filled in for exactly this consumer). The clamp depends on
            // geometry only, so once per lumel — not per light — suffices.
            if (f.clampToWorld) {
                ++stats.clampRays;
                RayHit clampHit;
                int32_t clampTerminal = -1;
                const bool clampBlocked =
                    raycastWorld(wr, centreOff, p, clampHit, &clampTerminal,
                                 static_cast<int32_t>(cellIdx),
                                 /*zeroEpsilon=*/true);
                if (clampBlocked ||
                    clampHit.status == RayStatus::DiscardNoPolygon) {
                    p = clampHit.point;
                    if (clampTerminal >= 0) pCell = clampTerminal;
                    ++stats.clampMoved;
                } else if (clampHit.status == RayStatus::Clear) {
                    // The terminal cell is the cell containing the (reached)
                    // lumel point — a better hint than the polygon's cell for
                    // lumels that poke past a portal.
                    if (clampTerminal >= 0) pCell = clampTerminal;
                } else {
                    ++stats.clampUnproven;   // keep the unclamped target
                }
            }

            LumelAccumulator acc;
            if (f.includeAmbient) acc.seedAmbient(rp.ambientLight, f);

            for (int k = 0; k < n && k + 1 < static_cast<int>(lt.size()); ++k) {
                const int32_t lightIdx = lt[k + 1];

                if (lightIdx == 0) {
                    // Slot 0 = sun. Its presence in the cell's list only says
                    // the CELL has sky access somewhere; this lumel needs its
                    // own trace to a sky face — cast from the CLAMPED point,
                    // as the original does. See bakeSun's note 3.
                    if (f.includeSun) {
                        ++stats.sunTests;
                        if (pointSeesSky(wr, p, rp, pCell)) {
                            ++stats.sunLit;
                            LightSample sun;
                            sun.contribution = bakeSun(rp, g.normal);
                            sun.preSpotPeak = std::max({sun.contribution.x,
                                                        sun.contribution.y,
                                                        sun.contribution.z});
                            acc.add(sun, 1.0f, f);
                            if (dirAccum && sun.preSpotPeak > 0.0f) {
                                DirAccum &da =
                                    fineDir[static_cast<size_t>(j) * g.lx + i];
                                da.sum += rp.sunlightNorm * sun.preSpotPeak;
                                da.weight += sun.preSpotPeak;
                            }
                        }
                    }
                    continue;
                }
                if (lightIdx >= static_cast<int32_t>(wr.staticLights.size()))
                    continue;
                if (lightMask && !lightMask[lightIdx]) continue;
                const WRStaticLight &L = wr.staticLights[lightIdx];

                // Cheap rejection before the expensive part. Not in throw
                // mode — the radius is spent as intensity there and the
                // sub-quantisation skip below bounds the cost instead.
                if (!f.throwMode() && L.radius > 0.0f) {
                    const Vector3 d = p - L.loc;
                    if (glm::dot(d, d) > L.radius * L.radius) continue;
                }

                // Evaluate the formula first — it is much cheaper than the
                // visibility rays, and a light this lumel faces away from
                // needs no ray at all (the original's per-polygon backface
                // gate is this same test: dot(L−P, n) IS the light's plane
                // distance for any P on the plane).
                const LightSample sample =
                    bakeOneLight(L, p, g.normal, f, lightIdx);
                if (sample.preSpotPeak <= 0.0f) continue;
                // Sub-quantisation skip: a contribution under half a 1/255
                // count stores nothing in ANY storage mode, so its ray is
                // pure cost. This is also what keeps reach expansion
                // affordable — imported far lights usually die right here.
                if (sample.preSpotPeak * sample.spotFactor * 255.0f < 0.5f)
                    continue;

                // Fraction of the emitter this lumel can see. 0 = fully
                // shadowed, 1 = fully lit, anything between is penumbra — the
                // term that makes the result smooth rather than stepped.
                if (inside) ++stats.raysInside; else ++stats.raysOutside;
                const bool dynOcc =
                    f.segmentBlockedFn && f.extraOverlayLights &&
                    static_cast<size_t>(lightIdx) <
                        f.extraOverlayLights->size() &&
                    (*f.extraOverlayLights)[lightIdx];
                const float visFrac = visibilityFraction(
                    wr, p, L.loc, pCell, lightCells[lightIdx], f, stats,
                    dynOcc);
                if (visFrac <= 0.0f) continue;

                acc.add(sample, visFrac, f);

                if (dirAccum) {
                    const float w =
                        sample.preSpotPeak * sample.spotFactor * visFrac;
                    if (w > 0.0f) {
                        const Vector3 toL = L.loc - p;
                        const float len = glm::length(toL);
                        if (len > 1e-6f) {
                            DirAccum &da =
                                fineDir[static_cast<size_t>(j) * g.lx + i];
                            da.sum += (toL / len) * w;
                            da.weight += w;
                        }
                    }
                }
            }

            fine[static_cast<size_t>(j) * g.lx + i] = acc.finalize(f);
            ++stats.lumels;
        }
    }

    // Box-filter the ss x ss footprint of each output lumel.
    out.assign(static_cast<size_t>(outW) * outH, Vector3(0.0f));
    if (outInsidePolygon)
        outInsidePolygon->assign(static_cast<size_t>(outW) * outH, 0);
    const float inv = 1.0f / static_cast<float>(ss * ss);
    for (int oy = 0; oy < outH; ++oy) {
        for (int ox = 0; ox < outW; ++ox) {
            Vector3 acc(0.0f);
            int insideCount = 0;
            for (int sy = 0; sy < ss; ++sy) {
                for (int sx = 0; sx < ss; ++sx) {
                    const size_t k = static_cast<size_t>(oy * ss + sy) * g.lx
                                   + (ox * ss + sx);
                    acc += fine[k];
                    if (!fineInside.empty() && fineInside[k]) ++insideCount;
                    if (dirAccum) {
                        DirAccum &dst =
                            (*dirAccum)[static_cast<size_t>(oy) * outW + ox];
                        dst.sum += fineDir[k].sum;
                        dst.weight += fineDir[k].weight;
                    }
                }
            }
            out[static_cast<size_t>(oy) * outW + ox] = acc * inv;
            if (outInsidePolygon)
                // An output lumel counts as on-surface if ANY sub-sample was —
                // a footprint straddling the polygon edge is still partly real.
                (*outInsidePolygon)[static_cast<size_t>(oy) * outW + ox] =
                    insideCount > 0 ? 1 : 0;
        }
    }
    return true;
}

// Blit one polygon's baked lumels into the atlas and re-fill its padding.
// Shared by the threaded and serial paths; each polygon owns a disjoint
// rectangle, which is what makes the threading lock-free.
inline void writeLumelsToAtlas(const std::vector<Vector3> &lumels,
                               const LmapEntry &e, AtlasTexture &out,
                               int density = 1) {
    for (int y = 0; y < e.pixelH; ++y) {
        for (int x = 0; x < e.pixelW; ++x) {
            const Vector3 &v = lumels[static_cast<size_t>(y) * e.pixelW + x];
            const size_t px = (static_cast<size_t>(e.pixelY + y) * out.size
                               + (e.pixelX + x)) * 4;
            if (px + 3 >= out.rgba.size()) continue;
            out.rgba[px + 0] = static_cast<uint8_t>(
                std::min(1.0f, std::max(0.0f, v.x)) * 255.0f);
            out.rgba[px + 1] = static_cast<uint8_t>(
                std::min(1.0f, std::max(0.0f, v.y)) * 255.0f);
            out.rgba[px + 2] = static_cast<uint8_t>(
                std::min(1.0f, std::max(0.0f, v.z)) * 255.0f);
            out.rgba[px + 3] = 255;
        }
    }
    // Same edge-clamp the shipped packer applies, so filtering at polygon
    // borders behaves identically between the two atlases.
    // The padding GAP scales with density (scaleAtlasPlacement multiplies the
    // whole packing), so the fill has to scale with it too. Filling only 2 when
    // the gap is 2*density leaves an unwritten black ring around every one of
    // the mission's ~26k lightmap rectangles, which reads on screen as exactly
    // the blocky grid this whole exercise exists to remove.
    fillEdgePadding(out.rgba, out.size, e.pixelX, e.pixelY, e.pixelW, e.pixelH,
                    2 * density);
}

inline void mergeBakeStats(BakeStats &dst, const BakeStats &src) {
    dst.lumels += src.lumels;   dst.rays += src.rays;
    dst.rayBlocked += src.rayBlocked;
    dst.penumbraTrivial += src.penumbraTrivial;
    dst.penumbraRefined += src.penumbraRefined;
    dst.rayRetried += src.rayRetried;
    dst.rayUnevaluable += src.rayUnevaluable;
    dst.sunTests += src.sunTests; dst.sunLit += src.sunLit;
    dst.lumelsInside += src.lumelsInside;
    dst.lumelsOutside += src.lumelsOutside;
    dst.raysInside += src.raysInside; dst.raysOutside += src.raysOutside;
    dst.unresolvedInside += src.unresolvedInside;
    dst.unresolvedOutside += src.unresolvedOutside;
    dst.clampRays += src.clampRays;
    dst.clampMoved += src.clampMoved;
    dst.clampUnproven += src.clampUnproven;
    dst.emitterRecell += src.emitterRecell;
    // lightsNoCell is set once per bake, deliberately not merged.
    dst.gatherRays += src.gatherRays;
    dst.gatherSky += src.gatherSky;
    dst.gatherUnproven += src.gatherUnproven;
    dst.lumelsGathered += src.lumelsGathered;
    dst.bounceSum255 += src.bounceSum255;
    dst.aoSumMil += src.aoSumMil;
    dst.dirRatioSumMil += src.dirRatioSumMil;
    dst.dirTexels += src.dirTexels;
    dst.geomCandidates += src.geomCandidates;
    dst.doorOverlays += src.doorOverlays;
    dst.geomProbeCulled += src.geomProbeCulled;
    dst.geomProbeRays += src.geomProbeRays;
    dst.roundtripFail += src.roundtripFail;
    for (int i = 0; i < 16; ++i) dst.byStatus[i] += src.byStatus[i];
}

// ── Whole-mission bake into an atlas ────────────────────────────────────────
//
// Fills `out` — a copy of the atlas the shipped lightmaps were packed into —
// with OUR computed values, reusing the existing `LmapEntry` placements so the
// mesh UVs built for the shipped atlas address ours unchanged. That is what
// makes an in-game A/B a texture swap rather than a rebuild.
//
// Measured 2026-08-06: 3.2 s single-threaded for MISS6 (792k lumels, 4.3M
// rays). Slow for a shipping path, fine for a debug view, and the reason a
// look at our own output does not have to wait for the GPU bake.
inline void bakeAtlasInPlace(const WRParsedData &wr,
                             const RenderParams &rp,
                             const BakeFormula &f,
                             const LightmapAtlasSet &placement,
                             AtlasTexture &out,
                             BakeStats &stats,
                             int threads = 0) {
    // Each light's containing cell, resolved once — the light→surface trace
    // starts there, exactly as the original resolves it once at registration.
    const std::vector<int32_t> lightCells =
        resolveLightCells(wr, &stats.lightsNoCell);

    // Cells are independent and write to disjoint atlas rectangles, so the
    // whole bake parallelises with no locking. Worth it because penumbra
    // sampling multiplies the ray count by the sample budget.
    if (threads <= 0)
        threads = std::max(1u, std::thread::hardware_concurrency());
    if (threads > 1) {
        const uint32_t n = static_cast<uint32_t>(
            std::min<size_t>(wr.numCells, placement.entries.size()));
        std::vector<BakeStats> partial(threads);
        std::vector<std::thread> pool;
        std::atomic<uint32_t> next{0};
        for (int t = 0; t < threads; ++t) {
            pool.emplace_back([&, t]() {
                std::vector<Vector3> lumels;
                for (;;) {
                    const uint32_t ci = next.fetch_add(1);
                    if (ci >= n) break;
                    const WRParsedCell &cell = wr.cells[ci];
                    for (int pi = 0; pi < cell.numTextured; ++pi) {
                        if (pi >= static_cast<int>(placement.entries[ci].size())) break;
                        const LmapEntry &e = placement.entries[ci][pi];
                        if (e.pixelW <= 0 || e.pixelH <= 0) continue;
                        if (!bakePolygon(wr, rp, ci, pi, f, lightCells,
                                         lumels, partial[t])) continue;
                        if (static_cast<int>(lumels.size()) != e.pixelW * e.pixelH) continue;
                        writeLumelsToAtlas(lumels, e, out, f.density);
                    }
                }
            });
        }
        for (auto &th : pool) th.join();
        for (const auto &ps : partial) mergeBakeStats(stats, ps);
        return;
    }

    std::vector<Vector3> lumels;
    for (uint32_t ci = 0; ci < wr.numCells && ci < placement.entries.size(); ++ci) {
        const WRParsedCell &cell = wr.cells[ci];
        for (int pi = 0; pi < cell.numTextured; ++pi) {
            if (pi >= static_cast<int>(placement.entries[ci].size())) break;
            const LmapEntry &e = placement.entries[ci][pi];
            if (e.pixelW <= 0 || e.pixelH <= 0) continue;   // fallback texel
            if (!bakePolygon(wr, rp, ci, pi, f, lightCells, lumels, stats)) continue;
            if (static_cast<int>(lumels.size()) != e.pixelW * e.pixelH) continue;

            writeLumelsToAtlas(lumels, e, out, f.density);
        }
    }
}

// ── Hemisphere gather: indirect bounce + ambient occlusion (S6) ─────────────
//
// One cosine-weighted gather over the completed DIRECT bake computes both
// modern terms at once (PLAN.HIGH_RES_SHADOWS.md §S6):
//
//  * bounce — single-bounce indirect light. With cosine-weighted sampling of
//    a Lambertian scene the Monte-Carlo estimator collapses to the plain
//    average of (albedo_hit × direct_hit) over the rays: the pdf's cosθ/π
//    cancels the rendering equation's cosθ and the reflector's 1/π. The
//    original bake is direct-only, which is why its unlit corners are pitch
//    black and light never wraps a doorway.
//  * ambient occlusion — the cosine-weighted fraction of the hemisphere open
//    within a short physical range, scaling the AMBIENT term so corners and
//    trim ground themselves instead of floating in flat ambient.
//
// The gather runs on OUTPUT lumels (no supersampling — indirect light is low
// frequency) after the direct atlas is complete, because rays land on OTHER
// polygons whose direct values must already exist.

// Occlusion counts only within this range (world units ≈ feet): the scale of
// corners, trim and furniture-sized geometry. Beyond it a hit is just "the
// other side of the room" and must not darken anything.
constexpr float kAORange = 6.0f;
// Gather rays stop here. Longer only adds cell-traversal cost — a bounce
// carried farther than this contributes below quantisation.
constexpr float kBounceRayLength = 120.0f;

// Per-texture-index mean albedo, 0..1 RGB. Index 0..255 matches the WR
// texture list; entries default to mid-grey so a bounce without texture data
// (headless) still behaves sanely.
struct AlbedoTable {
    Vector3 albedo[256];
    AlbedoTable() {
        for (auto &a : albedo) a = Vector3(0.5f);
    }
};

// xorshift32 → [0,1). Deterministic because a bake that changes between runs
// cannot be diffed against itself (same rule as emitterSampleOffset).
inline float bakeRand01(uint32_t &st) {
    st ^= st << 13; st ^= st >> 17; st ^= st << 5;
    return static_cast<float>(st) * (1.0f / 4294967296.0f);
}

// Deterministic cosine-weighted hemisphere direction around `normal`.
// Standard mapping: r=√u1, φ=2πu2, z=√(1−u1) — pdf cosθ/π.
inline Vector3 cosineHemisphereDir(uint32_t &rng, const Vector3 &normal) {
    const float u1 = bakeRand01(rng);
    const float u2 = bakeRand01(rng);
    const float r = std::sqrt(u1);
    const float phi = 6.2831853f * u2;
    const float x = r * std::cos(phi);
    const float y = r * std::sin(phi);
    const float z = std::sqrt(std::max(0.0f, 1.0f - u1));
    const Vector3 up = (std::abs(normal.z) < 0.9f) ? Vector3(0, 0, 1)
                                                   : Vector3(1, 0, 0);
    const Vector3 tx = glm::normalize(glm::cross(up, normal));
    const Vector3 ty = glm::cross(normal, tx);
    return tx * x + ty * y + normal * z;
}

// Locate the atlas texel of a world-space point on a KNOWN polygon, by
// inverting the same parameterisation buildLumelGrid encodes. Returns false
// for degenerate grids or points whose lumel falls outside the rect.
inline bool atlasTexelForPoint(const LumelGrid &g, const LmapEntry &e,
                               const Vector3 &point,
                               int &outX, int &outY) {
    if (!g.valid || g.stepU <= 0.0f || g.stepV <= 0.0f) return false;
    // Project the offset into the (possibly skewed) texture-axis basis —
    // the same Gram solve the grid was built from.
    const Vector3 d = point - g.origin;
    const float m2u = glm::length2(g.axisU);
    const float m2v = glm::length2(g.axisV);
    if (m2u < 1e-9f || m2v < 1e-9f) return false;
    const float dotp = glm::dot(g.axisU, g.axisV);
    const float pu = glm::dot(g.axisU, d);
    const float pv = glm::dot(g.axisV, d);
    float projU, projV;
    if (std::abs(dotp) < 1e-6f) {
        projU = pu / m2u;
        projV = pv / m2v;
    } else {
        const float corr = 1.0f / (m2u * m2v - dotp * dotp);
        projU = pu * (corr * m2v) - pv * (corr * dotp);
        projV = pv * (corr * m2u) - pu * (corr * dotp);
    }
    // LumelGrid::at(i,j) = origin + axisU·(baseU + stepU·i) + ... — invert.
    const int i = static_cast<int>(std::lround((projU - g.baseU) / g.stepU));
    const int j = static_cast<int>(std::lround((projV - g.baseV) / g.stepV));
    if (i < 0 || j < 0 || i >= e.pixelW || j >= e.pixelH) return false;
    outX = e.pixelX + i;
    outY = e.pixelY + j;
    return true;
}

// ── Overlay-structured bake (the runtime decomposition) ─────────────────────
//
// The shipped data stores lighting as STATIC LAYER + one overlay per animated
// light per polygon, blended at runtime as `static + Σ intensity·overlay`.
// A re-bake that collapses everything into one atlas freezes every animated
// light at full brightness — so the shippable bake must reproduce the same
// decomposition: a base layer from the non-animated lights (plus ambient and
// sun), and per-light overlays in the polygon's animflags bit order so the
// existing blend walk applies unchanged.
//
// The overlay bake relies on the table's `bright` for animated lights holding
// their effective MAXIMUM. On disk it does NOT (it is the save-time state —
// measured by lightmap_overlays); the renderer synthesizes the maximum and
// overwrites the table at load, before this runs. The headless harness never
// calls this driver.
struct RebakedAnimPoly {
    uint32_t ci = 0;
    int32_t  pi = 0;
    int      w = 0, h = 0;          // atlas rect dims at bake density
    // Pristine base crop (RGB8). The blend source — the atlas buffer itself
    // gets overwritten by blends, so the base must survive separately.
    std::vector<uint8_t> baseCrop;
    // One RGB8 buffer per overlay, in animflags bit order (lowest set bit
    // first) — the same order the shipped file stores and the blend walks.
    // An overlay whose animMap entry is out of range bakes empty (all zero).
    std::vector<std::vector<uint8_t>> overlays;
    // The static-table light behind each overlay (animMap[bit]) — which is
    // also the number runtime intensities are keyed by. -1 = unmapped.
    std::vector<int16_t> overlayLightIdx;
};

// Re-blend one polygon of OUR bake into the re-baked atlas:
//   result = base + Σ intensity·tint·overlay
// The re-baked twin of blendAnimatedLightmap, sourcing our RGB8 crops at bake
// density instead of the shipped 5:5:5 bytes. `entry` must come from the
// SCALED placement (its rect is rec.w × rec.h).
//
// `tints` (optional) carries per-light RGB multipliers — the Planckian
// colour ramp of the physical flicker (PLAN.FLICKER_PHYSICS.md): a dimming
// torch REDDENS, not just darkens. The vintage path never passes it.
// Encode a polygon's accumulated dominant-direction texels into the dir
// atlas (octa dir + directionality ratio; alpha untouched here — the
// load-time gather refines openness and runtime re-encodes must not
// clobber it). Shared by the load bake and the S4 door-event direction
// re-bake.
inline void writeDirTexelsToAtlas(const std::vector<DirAccum> &polyDir,
                                  const LmapEntry &e, AtlasTexture &dirAtlas,
                                  int density, bool preserveAlpha,
                                  uint64_t *ratioSumMil = nullptr,
                                  uint64_t *texels = nullptr) {
    for (int y = 0; y < e.pixelH; ++y) {
        for (int x = 0; x < e.pixelW; ++x) {
            const DirAccum &da =
                polyDir[static_cast<size_t>(y) * e.pixelW + x];
            float u = 0.5f, v = 0.5f, ratio = 0.0f;
            if (da.weight > 1e-6f) {
                const float mag = glm::length(da.sum);
                ratio = std::min(1.0f, mag / da.weight);
                octahedralEncode(
                    mag > 1e-6f ? da.sum / mag : Vector3(0.0f), u, v);
            }
            const size_t px = (static_cast<size_t>(e.pixelY + y)
                                   * dirAtlas.size + (e.pixelX + x)) * 4;
            if (px + 3 >= dirAtlas.rgba.size()) continue;
            dirAtlas.rgba[px + 0] = static_cast<uint8_t>(u * 255.0f);
            dirAtlas.rgba[px + 1] = static_cast<uint8_t>(v * 255.0f);
            dirAtlas.rgba[px + 2] = static_cast<uint8_t>(ratio * 255.0f);
            if (!preserveAlpha) dirAtlas.rgba[px + 3] = 255;
            if (ratioSumMil)
                *ratioSumMil += static_cast<uint64_t>(ratio * 1e6f);
            if (texels) ++(*texels);
        }
    }
    fillEdgePadding(dirAtlas.rgba, dirAtlas.size, e.pixelX, e.pixelY,
                    e.pixelW, e.pixelH, 2 * density);
}

inline void blendRebakedLightmap(const RebakedAnimPoly &rec,
                                 const LmapEntry &entry,
                                 const std::unordered_map<int16_t, float> &intensities,
                                 AtlasTexture &atlas,
                                 int density,
                                 const std::unordered_map<int16_t, Vector3> *tints
                                     = nullptr) {
    const size_t count = static_cast<size_t>(rec.w) * rec.h;
    if (rec.baseCrop.size() < count * 3) return;

    std::vector<float> blended(count * 3);
    for (size_t i = 0; i < count * 3; ++i)
        blended[i] = rec.baseCrop[i] / 255.0f;

    for (size_t k = 0; k < rec.overlays.size(); ++k) {
        float scale[3] = {1.0f, 1.0f, 1.0f};
        if (k < rec.overlayLightIdx.size()) {
            const int16_t lightIdx = rec.overlayLightIdx[k];
            auto it = intensities.find(lightIdx);
            const float intensity =
                (it != intensities.end()) ? it->second : 1.0f;
            scale[0] = scale[1] = scale[2] = intensity;
            if (tints) {
                auto tt = tints->find(lightIdx);
                if (tt != tints->end()) {
                    scale[0] *= tt->second.x;
                    scale[1] *= tt->second.y;
                    scale[2] *= tt->second.z;
                }
            }
        }
        if (scale[0] <= 0.0f && scale[1] <= 0.0f && scale[2] <= 0.0f)
            continue;
        const auto &ov = rec.overlays[k];
        if (ov.size() < count * 3) continue;
        for (size_t i = 0; i < count * 3; i += 3) {
            blended[i + 0] += scale[0] * (ov[i + 0] / 255.0f);
            blended[i + 1] += scale[1] * (ov[i + 1] / 255.0f);
            blended[i + 2] += scale[2] * (ov[i + 2] / 255.0f);
        }
    }

    for (int y = 0; y < rec.h && (entry.pixelY + y) < atlas.size; ++y) {
        for (int x = 0; x < rec.w && (entry.pixelX + x) < atlas.size; ++x) {
            const size_t src = (static_cast<size_t>(y) * rec.w + x) * 3;
            const size_t dst = (static_cast<size_t>(entry.pixelY + y) * atlas.size
                               + (entry.pixelX + x)) * 4;
            if (dst + 3 >= atlas.rgba.size()) continue;
            for (int c = 0; c < 3; ++c)
                atlas.rgba[dst + c] = static_cast<uint8_t>(
                    std::min(1.0f, std::max(0.0f, blended[src + c])) * 255.0f);
            atlas.rgba[dst + 3] = 255;
        }
    }
    // Same density-scaled padding rule as writeLumelsToAtlas — an unpadded
    // blend would reintroduce the black-ring artefact at every blended rect.
    fillEdgePadding(atlas.rgba, atlas.size, entry.pixelX, entry.pixelY,
                    rec.w, rec.h, 2 * density);
}

inline void packLumelsRGB8(const std::vector<Vector3> &lumels,
                           std::vector<uint8_t> &out) {
    out.resize(lumels.size() * 3);
    for (size_t i = 0; i < lumels.size(); ++i) {
        out[i * 3 + 0] = static_cast<uint8_t>(
            std::min(1.0f, std::max(0.0f, lumels[i].x)) * 255.0f);
        out[i * 3 + 1] = static_cast<uint8_t>(
            std::min(1.0f, std::max(0.0f, lumels[i].y)) * 255.0f);
        out[i * 3 + 2] = static_cast<uint8_t>(
            std::min(1.0f, std::max(0.0f, lumels[i].z)) * 255.0f);
    }
}

// The ambient value exactly as the accumulator would seed it — so a bake
// that composes ambient AFTER a gather matches one that seeded it, in the
// linear (Off/Continuous) storage modes the renderer uses.
inline Vector3 bakedAmbientValue(const Vector3 &ambient, const BakeFormula &f) {
    LumelAccumulator acc;
    acc.seedAmbient(ambient, f);
    return acc.finalize(f);
}

// Whole-mission bake, decomposed: `out` receives the BASE layer (non-animated
// lights + ambient + sun), `animPolys` one record per polygon that carries
// animated overlays. Threaded over cells like bakeAtlasInPlace.
//
// With `bounceSamples > 0` or `aoStrength > 0` the bake runs TWO phases:
// direct light into a scratch atlas first (ambient withheld), then a
// cosine-weighted hemisphere gather per output lumel over that completed
// atlas computes single-bounce indirect + ambient occlusion, and the final
// base composes direct + bounce + ambient·AO. Overlays stay direct-only —
// their indirect term is deferred (it would multiply the gather per light).
// `dirAtlas`, when non-null, must be sized like `out` and receives the
// dominant-direction field: octahedral direction in RG, directionality ratio
// in B, hemisphere openness in A (gather mode only; 255 otherwise).
inline void bakeAtlasWithOverlays(const WRParsedData &wr,
                                  const RenderParams &rp,
                                  const BakeFormula &f,
                                  const LightmapAtlasSet &placement,
                                  AtlasTexture &out,
                                  std::vector<RebakedAnimPoly> &animPolys,
                                  BakeStats &stats,
                                  int threads = 0,
                                  int bounceSamples = 0,
                                  float aoStrength = 0.0f,
                                  const AlbedoTable *albedo = nullptr,
                                  AtlasTexture *dirAtlas = nullptr) {
    const std::vector<int32_t> lightCells =
        resolveLightCells(wr, &stats.lightsNoCell);
    const std::vector<uint8_t> animated = animatedLightSet(wr);
    // Base mask: everything except the animated set. S3 door-adjacent
    // extras are excluded PER POLY instead (only where the poly actually
    // gets a door overlay — the shadow-cone gate): a global exclusion
    // paired with cone-gated overlays LOSES the light everywhere outside
    // the cones (measured: [LUM_RATIO] p5 collapsed 0.47 -> 0.18).
    // Conservation rule: a poly's base excludes exactly the lights it
    // carries overlays for.
    std::vector<uint8_t> baseMask(wr.staticLights.size(), 1);
    for (size_t i = 0; i < animated.size(); ++i)
        if (animated[i]) baseMask[i] = 0;

    // Throw-derived per-light anchors (intensity recovery). Owned here
    // unless the caller supplied its own table (the renderer does, so the
    // runtime door re-bake can reuse the identical anchors) —
    // baseF/overlayF below carry non-owning pointers, and the vector
    // outlives every worker thread this function spawns.
    std::vector<float> throwAnchors;
    const bool throwMode = f.falloffPhysical && f.throwAlpha > 0.0f;
    if (throwMode && !f.perLightAnchor)
        throwAnchors = physicalAnchors(wr, f.throwAlpha, f.falloffAnchor);

    // Overlay bakes carry exactly one light: no ambient, no sun, hard
    // shadows kept identical to the base so the two layers sum coherently.
    BakeFormula overlayF = f;
    overlayF.includeAmbient = false;
    overlayF.includeSun     = false;

    const bool gather = (bounceSamples > 0 || aoStrength > 0.0f);
    // Phase A bakes WITHOUT ambient when gathering: ambient is composed at
    // the end so AO can scale it, and so the gather reads pure direct light
    // (ambient bouncing off every surface would double-count as fake GI).
    BakeFormula baseF = f;
    if (gather) baseF.includeAmbient = false;

    const std::vector<float> *anchorTable =
        throwMode ? (f.perLightAnchor ? f.perLightAnchor : &throwAnchors)
                  : nullptr;
    if (throwMode) {
        overlayF.perLightAnchor = anchorTable;
        baseF.perLightAnchor    = anchorTable;
    }

    static const AlbedoTable kGreyAlbedo;
    const AlbedoTable &alb = albedo ? *albedo : kGreyAlbedo;

    // Light-reach determination. Physical mode: pure geometry — per-light
    // sub-quantisation radii; candidates gathered per POLYGON by sphere test
    // + occlusion probes below, authored lists unused. Original mode:
    // authored lists, optionally hop-widened.
    std::vector<float> reachRadii;
    std::vector<std::vector<uint16_t>> expandedLists;
    if (f.falloffPhysical) {
        reachRadii.resize(wr.staticLights.size(), 0.0f);
        for (size_t li = 1; li < wr.staticLights.size(); ++li)
            if (lightCells[li] >= 0)   // outside the world: casts nothing
                reachRadii[li] = physicalReachRadius(
                    wr.staticLights[li],
                    anchorTable ? (*anchorTable)[li] : f.falloffAnchor,
                    f.emitterRadius, f.brightScale,
                    /*clampToAuthored=*/!throwMode);
    } else if (f.reachExpand > 0) {
        expandedLists = expandCellLightLists(wr, f.reachExpand);
    }

    if (threads <= 0)
        threads = std::max(1u, std::thread::hardware_concurrency());
    const uint32_t n = static_cast<uint32_t>(
        std::min<size_t>(wr.numCells, placement.entries.size()));

    // The direct-light store the gather reads. Without a gather, Phase A
    // writes the final base straight into `out` (the pre-S6 behaviour).
    AtlasTexture direct;
    if (gather) {
        direct.size = out.size;
        direct.rgba.assign(out.rgba.size(), 0);
    }
    AtlasTexture &phaseATarget = gather ? direct : out;

    // Per-polygon lumel grids at OUTPUT density — the gather locates hit
    // texels through these, and building one per ray would double the cost.
    std::vector<std::vector<LumelGrid>> grids;
    if (gather) {
        grids.resize(n);
        for (uint32_t ci = 0; ci < n; ++ci) {
            const WRParsedCell &cell = wr.cells[ci];
            grids[ci].resize(cell.numTextured);
            for (int pi = 0; pi < cell.numTextured; ++pi)
                grids[ci][pi] = buildLumelGrid(wr, ci, pi, f.density);
        }
    }

    std::vector<BakeStats> partial(std::max(1, threads));
    std::vector<std::vector<RebakedAnimPoly>> partialAnim(std::max(1, threads));
    std::vector<std::thread> pool;
    std::atomic<uint32_t> next{0};

    // ── Phase A: direct light (+ overlays), threaded over cells ──
    auto workerA = [&](int t) {
        std::vector<Vector3> lumels;
        std::vector<uint8_t> overlayMask(wr.staticLights.size(), 0);
        std::vector<DirAccum> polyDir;
        std::vector<uint16_t> polyCandidates;
        for (;;) {
            const uint32_t ci = next.fetch_add(1);
            if (ci >= n) break;
            const WRParsedCell &cell = wr.cells[ci];
            for (int pi = 0; pi < cell.numTextured; ++pi) {
                if (pi >= static_cast<int>(placement.entries[ci].size())) break;
                const LmapEntry &e = placement.entries[ci][pi];
                if (e.pixelW <= 0 || e.pixelH <= 0) continue;

                std::vector<DirAccum> *dirPtr = nullptr;
                if (dirAtlas) {
                    polyDir.assign(
                        static_cast<size_t>(e.pixelW) * e.pixelH, DirAccum{});
                    dirPtr = &polyDir;
                }

                // Candidate lights. Physical mode: geometry only — the
                // polygon's bounding sphere against each light's reach
                // sphere, then occlusion probes from the polygon's centre
                // and vertices (a light invisible to the WHOLE polygon
                // casts nothing on it; per-lumel rays handle everything
                // partial). Authored lists play no role here.
                const std::vector<uint16_t> *lightList = nullptr;
                if (f.falloffPhysical) {
                    const auto &pidx = cell.polyIndices[pi];
                    Vector3 pc(0.0f);
                    for (uint8_t vi : pidx) pc += cell.vertices[vi];
                    pc /= static_cast<float>(pidx.size());
                    float pr = 0.0f;
                    for (uint8_t vi : pidx)
                        pr = std::max(pr,
                                      glm::length(cell.vertices[vi] - pc));
                    const Vector3 pn =
                        (cell.polygons[pi].plane < cell.planes.size())
                            ? cell.planes[cell.polygons[pi].plane].normal
                            : Vector3(0, 0, 1);

                    polyCandidates.clear();
                    polyCandidates.push_back(0);
                    // The sun keeps its authored per-cell presence: sky
                    // access is gated per lumel by a real ray anyway.
                    {
                        const auto &own = cell.lightIndices;
                        const int cnt =
                            own.empty() ? 0 : static_cast<int>(own[0]);
                        for (int k = 1;
                             k <= cnt && k < static_cast<int>(own.size());
                             ++k)
                            if (own[k] == 0) {
                                polyCandidates.push_back(0);
                                break;
                            }
                    }
                    for (size_t li = 1; li < wr.staticLights.size(); ++li) {
                        const float rq = reachRadii[li];
                        if (rq <= 0.0f) continue;
                        const WRStaticLight &L = wr.staticLights[li];
                        const float d = glm::length(L.loc - pc);
                        if (d > rq + pr) continue;
                        ++partial[t].geomCandidates;
                        // Backface: the plane distance of the light is the
                        // cosine term for every lumel at once.
                        if (glm::dot(L.loc - pc, pn) < 0.0f) continue;
                        // Occlusion probes: centre + vertices, offset into
                        // the cell's air. Skip only when ALL are blocked —
                        // a light lighting only a pinhole smaller than the
                        // vertex spread is lost, at sub-visible cost.
                        bool anyVisible = false;
                        for (size_t pv = 0; pv <= pidx.size() && !anyVisible;
                             ++pv) {
                            const Vector3 probe =
                                ((pv == 0) ? pc : cell.vertices[pidx[pv - 1]])
                                + pn * f.surfaceOffset;
                            ++partial[t].geomProbeRays;
                            VisibilityResult v;
                            if (f.traceFromLight) {
                                v = resolveVisibilityFromLight(
                                    wr, L.loc, probe,
                                    lightCells[li]);
                            } else {
                                v = resolveVisibility(
                                    wr, probe, L.loc,
                                    static_cast<int32_t>(ci), f);
                            }
                            anyVisible = (v == VisibilityResult::Clear);
                        }
                        if (!anyVisible) {
                            ++partial[t].geomProbeCulled;
                            continue;
                        }
                        polyCandidates.push_back(
                            static_cast<uint16_t>(li));
                    }
                    polyCandidates[0] = static_cast<uint16_t>(
                        polyCandidates.size() - 1);
                    lightList = &polyCandidates;
                } else if (!expandedLists.empty()) {
                    lightList = &expandedLists[ci];
                }

                // S3: compute this poly's door-overlay lights BEFORE the
                // base bake — the per-poly base mask excludes exactly them.
                std::vector<int16_t> extraForPoly;
                if (f.extraOverlayLights && f.falloffPhysical && lightList &&
                    f.doorSpheres && !f.doorSpheres->empty()) {
                    const auto &pidx2 = cell.polyIndices[pi];
                    Vector3 pc2(0.0f);
                    for (uint8_t vi : pidx2) pc2 += cell.vertices[vi];
                    pc2 /= static_cast<float>(pidx2.size());
                    float pr2 = 0.0f;
                    for (uint8_t vi : pidx2)
                        pr2 = std::max(pr2,
                                       glm::length(cell.vertices[vi] - pc2));
                    const auto &ll = *lightList;
                    const int cnt =
                        ll.empty() ? 0 : static_cast<int>(ll[0]);
                    for (int k = 1;
                         k <= cnt && k < static_cast<int>(ll.size()); ++k) {
                        const uint16_t li = ll[k];
                        if (!(li > 0 &&
                              li < f.extraOverlayLights->size() &&
                              (*f.extraOverlayLights)[li]))
                            continue;
                        if (baseMask[li] == 0) continue;  // already animated
                        bool coneHit = false;
                        for (const auto &ds : *f.doorSpheres) {
                            if (segmentNearSphere(
                                    pc2, wr.staticLights[li].loc,
                                    Vector3(ds.x, ds.y, ds.z),
                                    ds.w + pr2)) {
                                coneHit = true;
                                break;
                            }
                        }
                        if (coneHit)
                            extraForPoly.push_back(
                                static_cast<int16_t>(li));
                    }
                }
                const uint8_t *polyBaseMask = baseMask.data();
                std::vector<uint8_t> maskedBase;
                if (!extraForPoly.empty()) {
                    maskedBase = baseMask;
                    for (int16_t xli : extraForPoly)
                        maskedBase[static_cast<size_t>(xli)] = 0;
                    polyBaseMask = maskedBase.data();
                }

                if (!bakePolygon(wr, rp, ci, pi, baseF, lightCells, lumels,
                                 partial[t], nullptr, polyBaseMask, dirPtr,
                                 lightList))
                    continue;
                if (static_cast<int>(lumels.size()) != e.pixelW * e.pixelH)
                    continue;
                writeLumelsToAtlas(lumels, e, phaseATarget, f.density);

                const uint32_t animflags =
                    (pi < static_cast<int>(cell.lightInfos.size()))
                        ? cell.lightInfos[pi].animflags : 0;
                if (animflags || !extraForPoly.empty()) {
                    RebakedAnimPoly rec;
                    rec.ci = ci;
                    rec.pi = pi;
                    rec.w = e.pixelW;
                    rec.h = e.pixelH;
                    // Without a gather this IS the final base; with one,
                    // Phase B overwrites it after composing.
                    packLumelsRGB8(lumels, rec.baseCrop);

                    uint32_t flags = animflags;
                    while (flags) {
                        const int bit = __builtin_ctz(flags);
                        flags &= flags - 1;
                        const int16_t lightIdx =
                            (bit < static_cast<int>(cell.animMap.size()))
                                ? cell.animMap[bit] : -1;
                        std::vector<uint8_t> buf;
                        if (lightIdx > 0 &&
                            static_cast<size_t>(lightIdx) <
                                wr.staticLights.size()) {
                            std::fill(overlayMask.begin(), overlayMask.end(),
                                      0);
                            overlayMask[lightIdx] = 1;
                            // Overlays join the DIRECTION field too — a
                            // torch's direction is real even though its
                            // energy animates.
                            if (bakePolygon(wr, rp, ci, pi, overlayF,
                                            lightCells, lumels, partial[t],
                                            nullptr, overlayMask.data(),
                                            dirPtr, lightList) &&
                                static_cast<int>(lumels.size()) ==
                                    e.pixelW * e.pixelH) {
                                packLumelsRGB8(lumels, buf);
                            }
                        }
                        if (buf.empty())
                            buf.assign(
                                static_cast<size_t>(e.pixelW) * e.pixelH * 3,
                                0);
                        rec.overlays.push_back(std::move(buf));
                        rec.overlayLightIdx.push_back(lightIdx);
                    }
                    // S3 extras: appended AFTER the animflags bits (whose
                    // order the blend and the shipped format both fix), one
                    // overlay per door-adjacent candidate not already
                    // recorded via animMap. The blend scales unmapped
                    // lights at intensity 1.0 — geometry-only overlays.
                    // Door overlays: energy AND direction bake with the
                    // door occlusion hook — the direction field reflects
                    // the CURRENT door state, and door events re-bake it
                    // alongside the energy (a door-transparent direction
                    // was tried and leaked specular through closed doors:
                    // light.rgb is TOTAL energy, so ambient fed a lobe
                    // pointing at the hidden light).
                    for (int16_t xli : extraForPoly) {
                        bool already = false;
                        for (int16_t seen : rec.overlayLightIdx)
                            if (seen == xli) { already = true; break; }
                        if (already) continue;
                        std::fill(overlayMask.begin(), overlayMask.end(), 0);
                        overlayMask[static_cast<size_t>(xli)] = 1;
                        std::vector<uint8_t> buf;
                        if (bakePolygon(wr, rp, ci, pi, overlayF,
                                        lightCells, lumels, partial[t],
                                        nullptr, overlayMask.data(),
                                        dirPtr, lightList) &&
                            static_cast<int>(lumels.size()) ==
                                e.pixelW * e.pixelH) {
                            packLumelsRGB8(lumels, buf);
                        }
                        if (buf.empty())
                            buf.assign(
                                static_cast<size_t>(e.pixelW) * e.pixelH * 3,
                                0);
                        rec.overlays.push_back(std::move(buf));
                        rec.overlayLightIdx.push_back(xli);
                        ++partial[t].doorOverlays;
                    }
                    partialAnim[t].push_back(std::move(rec));
                }

                // Encode the dominant-direction texels now that base AND
                // overlays have accumulated. Alpha starts at 255 (fully
                // open); the gather phase refines it with real openness.
                if (dirAtlas)
                    writeDirTexelsToAtlas(polyDir, e, *dirAtlas, f.density,
                                          /*preserveAlpha=*/false,
                                          &partial[t].dirRatioSumMil,
                                          &partial[t].dirTexels);
            }
        }
    };

    if (threads > 1) {
        for (int t = 0; t < threads; ++t) pool.emplace_back(workerA, t);
        for (auto &th : pool) th.join();
        pool.clear();
    } else {
        workerA(0);
    }
    for (auto &pa : partialAnim)
        for (auto &rec : pa) animPolys.push_back(std::move(rec));

    if (!gather) {
        for (const auto &ps : partial) mergeBakeStats(stats, ps);
        return;
    }

    // ── Phase B: hemisphere gather + compose, threaded over cells ──
    // Barrier above is REQUIRED: the gather reads other polygons' direct
    // values, so the whole direct atlas must exist first.
    std::unordered_map<uint64_t, RebakedAnimPoly *> animIndex;
    for (auto &rec : animPolys)
        animIndex[(static_cast<uint64_t>(rec.ci) << 32)
                  | static_cast<uint32_t>(rec.pi)] = &rec;

    const Vector3 ambientVal = f.includeAmbient
        ? bakedAmbientValue(rp.ambientLight, f) : Vector3(0.0f);

    auto readDirect = [&](int x, int y) {
        const size_t px = (static_cast<size_t>(y) * direct.size + x) * 4;
        return Vector3(direct.rgba[px + 0] / 255.0f,
                       direct.rgba[px + 1] / 255.0f,
                       direct.rgba[px + 2] / 255.0f);
    };

    next.store(0);
    auto workerB = [&](int t) {
        BakeStats &st = partial[t];
        std::vector<Vector3> lumels;
        for (;;) {
            const uint32_t ci = next.fetch_add(1);
            if (ci >= n) break;
            const WRParsedCell &cell = wr.cells[ci];
            for (int pi = 0; pi < cell.numTextured; ++pi) {
                if (pi >= static_cast<int>(placement.entries[ci].size())) break;
                const LmapEntry &e = placement.entries[ci][pi];
                if (e.pixelW <= 0 || e.pixelH <= 0) continue;
                const LumelGrid &g = grids[ci][pi];
                if (!g.valid || g.lx != e.pixelW || g.ly != e.pixelH) continue;

                // Round-trip self-check: the grid's own lumel centres must
                // locate to their own texels, or every gather sample this
                // polygon receives is being misdirected.
                {
                    int rx = 0, ry = 0;
                    if (!atlasTexelForPoint(g, e, g.at(0, 0), rx, ry) ||
                        rx != e.pixelX || ry != e.pixelY)
                        ++st.roundtripFail;
                }

                lumels.assign(static_cast<size_t>(e.pixelW) * e.pixelH,
                              Vector3(0.0f));
                for (int j = 0; j < e.pixelH; ++j) {
                    for (int i = 0; i < e.pixelW; ++i) {
                        const Vector3 dval = readDirect(e.pixelX + i,
                                                        e.pixelY + j);
                        const Vector3 p = g.at(i, j)
                                        + g.normal * f.surfaceOffset;
                        uint32_t rng = (ci * 7919u) ^ (pi * 977u)
                                     ^ (static_cast<uint32_t>(j) * 131071u)
                                     ^ static_cast<uint32_t>(i) ^ 0xB07CE5u;
                        if (rng == 0) rng = 0xB07CE5u;
                        Vector3 bounce(0.0f);
                        int open = 0;
                        const int rays = std::max(1, bounceSamples);
                        for (int r = 0; r < rays; ++r) {
                            const Vector3 dir =
                                cosineHemisphereDir(rng, g.normal);
                            RayHit hit;
                            int32_t term = -1;
                            ++st.gatherRays;
                            const bool blocked = raycastWorld(
                                wr, p, p + dir * kBounceRayLength, hit, &term,
                                static_cast<int32_t>(ci));
                            if (blocked) {
                                if (hit.textureIndex >= kSkyTextureIndex) {
                                    ++st.gatherSky;
                                    ++open;
                                    continue;
                                }
                                if (hit.distance >= kAORange) ++open;
                                if (bounceSamples > 0 &&
                                    hit.cellIdx >= 0 && hit.polyIdx >= 0 &&
                                    hit.cellIdx < static_cast<int32_t>(n) &&
                                    hit.polyIdx < static_cast<int32_t>(
                                        grids[hit.cellIdx].size()) &&
                                    hit.polyIdx < static_cast<int32_t>(
                                        placement.entries[hit.cellIdx].size())) {
                                    const LumelGrid &hg =
                                        grids[hit.cellIdx][hit.polyIdx];
                                    const LmapEntry &he =
                                        placement.entries[hit.cellIdx]
                                                         [hit.polyIdx];
                                    int hx = 0, hy = 0;
                                    if (hg.valid && he.pixelW > 0 &&
                                        atlasTexelForPoint(hg, he, hit.point,
                                                           hx, hy)) {
                                        const uint8_t tex =
                                            (hit.textureIndex >= 0 &&
                                             hit.textureIndex < 256)
                                                ? static_cast<uint8_t>(
                                                      hit.textureIndex)
                                                : 0;
                                        const Vector3 a = alb.albedo[tex];
                                        const Vector3 dv = readDirect(hx, hy);
                                        bounce += Vector3(a.x * dv.x,
                                                          a.y * dv.y,
                                                          a.z * dv.z);
                                    }
                                }
                            } else {
                                // Clear or unproven: nothing hit within
                                // range — open sky-less air either way.
                                if (!rayStatusProven(hit.status))
                                    ++st.gatherUnproven;
                                ++open;
                            }
                        }
                        const float openness =
                            static_cast<float>(open) / rays;
                        if (bounceSamples > 0)
                            bounce /= static_cast<float>(rays);
                        const float aoTerm =
                            1.0f - aoStrength * (1.0f - openness);

                        lumels[static_cast<size_t>(j) * e.pixelW + i] =
                            dval + bounce + ambientVal * aoTerm;

                        ++st.lumelsGathered;
                        const float bpk =
                            std::max({bounce.x, bounce.y, bounce.z});
                        st.bounceSum255 += static_cast<uint64_t>(
                            std::min(255.0f, bpk * 255.0f));
                        st.aoSumMil += static_cast<uint64_t>(
                            openness * 1e6f);

                        // Real hemisphere openness into the direction
                        // atlas's alpha (Phase A seeded it fully open).
                        if (dirAtlas) {
                            const size_t px =
                                (static_cast<size_t>(e.pixelY + j)
                                     * dirAtlas->size + (e.pixelX + i)) * 4;
                            if (px + 3 < dirAtlas->rgba.size())
                                dirAtlas->rgba[px + 3] =
                                    static_cast<uint8_t>(openness * 255.0f);
                        }
                    }
                }
                writeLumelsToAtlas(lumels, e, out, f.density);
                if (dirAtlas)
                    fillEdgePadding(dirAtlas->rgba, dirAtlas->size, e.pixelX,
                                    e.pixelY, e.pixelW, e.pixelH,
                                    2 * f.density);

                auto it = animIndex.find(
                    (static_cast<uint64_t>(ci) << 32)
                    | static_cast<uint32_t>(pi));
                if (it != animIndex.end())
                    packLumelsRGB8(lumels, it->second->baseCrop);
            }
        }
    };

    if (threads > 1) {
        for (int t = 0; t < threads; ++t) pool.emplace_back(workerB, t);
        for (auto &th : pool) th.join();
    } else {
        workerB(0);
    }
    for (const auto &ps : partial) mergeBakeStats(stats, ps);
}

// ── Shipped-data reference ──────────────────────────────────────────────────

// The shipped static layer plus every animated overlay at full intensity —
// the state `bakePolygon` reproduces with no light mask. Values are 0..1 per
// channel, clamped the same way blendAnimatedLightmap clamps them.
//
// `withOverlays = false` returns the static layer alone — the comparison
// target for a bake that masks the animated lights out. That pairing avoids
// the animated-brightness ambiguity entirely (the table's `bright` for an
// animated light is its SAVE-TIME state, not what the overlay encodes —
// measured by lightmap_overlays).
inline bool shippedLumelsAllOn(const WRParsedData &wr,
                               uint32_t cellIdx, int polyIdx,
                               std::vector<Vector3> &out,
                               bool withOverlays = true) {
    if (cellIdx >= wr.numCells) return false;
    const WRParsedCell &cell = wr.cells[cellIdx];
    if (polyIdx < 0 || polyIdx >= static_cast<int>(cell.lightInfos.size()))
        return false;
    const WRLightInfo &li = cell.lightInfos[polyIdx];
    if (li.lx <= 0 || li.ly <= 0) return false;

    const size_t count = static_cast<size_t>(li.lx) * li.ly;
    out.assign(count, Vector3(0.0f));

    auto accumulate = [&](const std::vector<uint8_t> &src) {
        if (src.size() < count * static_cast<size_t>(wr.lightSize)) return;
        for (size_t p = 0; p < count; ++p) {
            uint8_t r, g, b;
            convertLmPixel(&src[p * wr.lightSize], wr.lightSize, r, g, b);
            out[p] += Vector3(r / 255.0f, g / 255.0f, b / 255.0f);
        }
    };

    accumulate(cell.staticLightmaps[polyIdx]);
    if (withOverlays)
        for (const auto &overlay : cell.animLightmaps[polyIdx])
            accumulate(overlay);

    for (auto &v : out) {
        v.x = std::min(1.0f, v.x);
        v.y = std::min(1.0f, v.y);
        v.z = std::min(1.0f, v.z);
    }
    return true;
}

// One shipped animated overlay, decoded to 0..1 RGB. `overlayIdx` counts in
// the file's storage order — the bit order of the polygon's animflags, lowest
// set bit first — which is also the order blendAnimatedLightmap walks.
inline bool shippedOverlayLumels(const WRParsedData &wr,
                                 uint32_t cellIdx, int polyIdx,
                                 int overlayIdx,
                                 std::vector<Vector3> &out) {
    if (cellIdx >= wr.numCells) return false;
    const WRParsedCell &cell = wr.cells[cellIdx];
    if (polyIdx < 0 || polyIdx >= static_cast<int>(cell.lightInfos.size()))
        return false;
    const WRLightInfo &li = cell.lightInfos[polyIdx];
    if (li.lx <= 0 || li.ly <= 0) return false;
    const auto &overlays = cell.animLightmaps[polyIdx];
    if (overlayIdx < 0 || overlayIdx >= static_cast<int>(overlays.size()))
        return false;

    const size_t count = static_cast<size_t>(li.lx) * li.ly;
    const std::vector<uint8_t> &src = overlays[overlayIdx];
    if (src.size() < count * static_cast<size_t>(wr.lightSize)) return false;
    out.assign(count, Vector3(0.0f));
    for (size_t p = 0; p < count; ++p) {
        uint8_t r, g, b;
        convertLmPixel(&src[p * wr.lightSize], wr.lightSize, r, g, b);
        out[p] = Vector3(r / 255.0f, g / 255.0f, b / 255.0f);
    }
    return true;
}

} // namespace Darkness
