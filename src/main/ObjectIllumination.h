/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// Per-object illumination computation. Faithful port of the Dark Engine's
// per-frame object lighting pass:
//
//   total = ambient + extraLight
//   for each visible static light in cell.lightIndices:
//       total += light.bright × distFalloff × spotFalloff
//   for each dynamic light (with fresh portal raycast):
//       total += light.bright × distFalloff
//
// The set of "visible" static lights is cached per object — visibility only
// changes when the object crosses a cell boundary or moves far enough to
// reveal/occlude lights. Caching makes the per-frame cost negligible for
// stationary objects (the common case) while still tracking moving objects
// like doors and elevators correctly.
//
// Output is RGB (vec3); the shader multiplies the texture/material color by
// this value to produce the final lit fragment.

#pragma once

#include "DarknessMath.h"
#include "WRChunkParser.h"
#include "RenderParamsParser.h"
#include "RayCaster.h"
#include "DynamicLightList.h"
#include "property/DarkPropertyDefs.h"
#include "property/TypedProperty.h"
#include "../../shaders/object_lighting_constants.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Darkness {

// Per-object light cap shared between the C++ array-build and the
// per-vertex shaders. See shaders/object_lighting_constants.h for the
// rationale and the cost analysis.
constexpr int kObjectLightCap = OBJECT_LIGHT_CAP;

// What buildLightArray() emits for one object's per-frame draw call.
// Layout matches what the per-vertex shaders' uniform arrays expect.
// SoA (struct-of-arrays) so each `loc` / `dir` / `bright` array is
// contiguous in memory — `bgfx::setUniform` walks an array as N × 16-byte
// vec4s, so the three sub-arrays MUST each be tightly packed (no
// interleaving). An AoS layout breaks this: bgfx would walk past the
// first element into the next field instead of the next array slot,
// scrambling the data and (when crossing the buffer boundary) faulting
// on a write into protected memory.
//
// Per-light slot semantics (one i across all three arrays):
//   loc[i]    .xyz = world position of the light
//             .w   = cos(inner) for spotlight; -1.0 sentinel = omni
//   dir[i]    .xyz = spotlight direction (zero for omni)
//             .w   = cos(outer) for spotlight; unused when loc.w == -1
//   bright[i] .rgb = pre-scaled radiant intensity (LightColor × multiplier)
//             .w   = radius²; 0 = no distance cutoff
//
// "Pre-scaled" means the C++ side has already applied the per-light
// multiplier from the animated-light system — the shader doesn't know
// about animation.
//
// Only the prefix [0, count) is meaningful; entries past `count` are
// untouched between frames and the shader is gated by `count`.
struct GPULightArray {
    int       count = 0;
    Vector3   ambient = Vector3(0.0f);
    glm::vec4 loc[kObjectLightCap];
    glm::vec4 dir[kObjectLightCap];
    glm::vec4 bright[kObjectLightCap];
};

// Per-object cache of static-light visibility within the object's cell.
// Each bit in `bits` corresponds to an entry in cell.lightIndices[1..N]:
// 1 = visible (contributes), 0 = occluded by world geometry. Cleared and
// rebuilt when the object's cell changes or it moves more than the polling
// epsilon.
struct ShadowCache {
    int32_t  cellId    = -1;            // last known cell, -1 = invalid
    Vector3  pos       = Vector3(0.0f); // last evaluated position
    uint32_t bits[3]   = { 0, 0, 0 };   // 96-bit mask, matches MAX_STATIC/8
    bool     valid     = false;
};

// Distance below which a moving object is considered "stationary" for cache
// purposes. Cache rebuilds beyond this delta. Tuned to be loose enough that
// idle physics jitter doesn't thrash the cache, but tight enough that an
// object stepping into a new lit area updates promptly.
constexpr float kShadowCacheRepollEps = 0.05f;

// Distance the sun is conceptually placed away from each object, in world
// units. Combined with the 1/r falloff this yields a small but non-zero
// contribution from very bright sun values (the editor field range is
// 0..1023). Matches the original engine's SUNLIGHT_DISTANCE constant.
constexpr float kSunlightDistance = 125.0f;

class ObjectIlluminator {
public:
    void setMissionData(const WRParsedData *wr,
                        const RenderParams *rp,
                        PropertyService   *propSvc) {
        mWr = wr;
        mRp = rp;
        mPropSvc = propSvc;
        mCache.clear();
        // Per-light brightness multiplier — defaults to 1.0 (full intensity
        // per the disk-loaded static value). Animated lights (P$AnimLight)
        // drive these per-frame so torches blinking on/off are reflected on
        // nearby objects. See setLightMultiplier() / resetLightMultipliers().
        mLightMultiplier.assign(wr ? wr->staticLights.size() : 0, 1.0f);
    }

    // Bind the per-frame dynamic light list. Caller (the render loop) is
    // expected to reset() it once per frame and then have gameplay systems
    // (player flashlight, fire arrows, mage spells, …) push lights into it
    // before any object is drawn.
    void setDynamicLights(const DynamicLightList *dyn) { mDyn = dyn; }

    // Set the brightness multiplier for one static-light slot. Used by the
    // animated lighting system to propagate AnimLight intensity changes
    // (FLICKER mode, brighten/dim transitions, on/off via tweqs) to the
    // per-object lighting path. Multiplier 0.0 effectively turns the light
    // off for object lighting; 1.0 = full disk-loaded brightness.
    void setLightMultiplier(int32_t lightIdx, float multiplier) {
        if (lightIdx < 0 || lightIdx >= static_cast<int32_t>(mLightMultiplier.size()))
            return;
        mLightMultiplier[lightIdx] = multiplier;
    }
    void resetLightMultipliers() {
        std::fill(mLightMultiplier.begin(), mLightMultiplier.end(), 1.0f);
    }

    // Count of static-light slots currently driven below ~99% intensity (off
    // or mid-fade). Flips up when a switch turns torches off so we can
    // confirm the chain at runtime. Watching this count change after a frob
    // verifies that animated-light state is reaching the per-object path.
    int dimmedLightCount() const {
        int n = 0;
        for (float m : mLightMultiplier)
            if (m < 0.99f) ++n;
        return n;
    }

    void clear() { mCache.clear(); }

    // Drop the cache entry for one object; safe to call even if the object
    // has never been evaluated. Future invalidation hooks (Phase 5) call this
    // when an object's position is mutated by gameplay (door open, tweq,
    // platform move, frob grab/release).
    void invalidate(int32_t objId) { mCache.erase(objId); }

    // Compute the per-object RGB tint at `pos` for object `objId`. `radius`
    // is half the largest extent of the object's bounding box (only used to
    // offset the virtual sun position so a large object doesn't have its sun
    // "inside" itself). `cellHint` skips the cell-find step when the caller
    // already knows the cell; pass -1 to look it up.
    Vector3 compute(int32_t objId,
                    const Vector3 &pos,
                    float radius,
                    int32_t cellHint = -1);

    // Build the per-object GPU light array for the per-vertex Lambertian
    // path. Mirrors `compute()` for visibility (cell light list, shadow
    // cache, dynamic-light raycast) but emits the array of lights the
    // shader will iterate per vertex, *without* folding cosine or 1/dist.
    //
    // Output is written into `out`. The shader reads `out.count` and
    // `out.ambient`; entries `[0, count)` of `out.lights` are valid. The
    // tail beyond `count` is left undefined; the shader is gated by
    // `count` and never reads past it.
    //
    // ExtraLight in additive-override mode (`isAdditive == 0`) still
    // forces ambient-only output: the array is emitted with `count = 0`,
    // so the shader sums no per-light contributions. This matches the
    // scalar path's early-return behavior.
    void buildLightArray(int32_t objId,
                         const Vector3 &pos,
                         float radius,
                         int32_t cellHint,
                         GPULightArray &out);

private:
    // Returns true if the static light at index `lightIdx` is visible from
    // `from` within cell `cellHint`. Sun (index 0) is treated as always
    // visible if it's listed in the cell's light_indices — the original
    // engine pre-computed this at level bake.
    bool locationSeesLight(const Vector3 &from,
                           int32_t cellHint,
                           int32_t lightIdx) const;

    // Add the contribution of one light to `total`, mirroring the engine's
    // per-light evaluation: distance attenuation × spotlight cone falloff.
    // Slot 0 (sun) is special-cased: the caller patches its loc/bright before
    // calling, but we still apply the same 1/distance falloff.
    void applyOneLight(int32_t lightIdx,
                       const Vector3 &pos,
                       Vector3 &total) const;

    const WRParsedData     *mWr      = nullptr;
    const RenderParams     *mRp      = nullptr;
    PropertyService        *mPropSvc = nullptr;
    const DynamicLightList *mDyn     = nullptr;
    std::vector<float>      mLightMultiplier;  // per static-light brightness scale

    // Sun slot patched per-evaluation (`compute()` writes loc/bright before
    // applying the contribution). Held as a mutable member rather than a
    // local so the patched values survive into helper methods. The actual
    // table lives in `mWr->staticLights`; we never mutate that copy.
    mutable WRStaticLight mSunSlot{};

    std::unordered_map<int32_t, ShadowCache> mCache;
};

// ── Inline implementation ──────────────────────────────────────────────────

inline Vector3 ObjectIlluminator::compute(int32_t objId,
                                           const Vector3 &pos,
                                           float radius,
                                           int32_t cellHint) {
    if (!mWr || !mRp) return Vector3(0.0f);

    // 1. Resolve cell. compute() is called for every visible object every
    //    frame, so the caller normally passes a precomputed hint; only walk
    //    the cell tree when nothing is cached.
    int32_t cell = (cellHint >= 0 && cellHint < static_cast<int32_t>(mWr->numCells))
                 ? cellHint
                 : findCameraCell(*mWr, pos.x, pos.y, pos.z);

    // 2. Resolve ambient + P$ExtraLight. Ambient comes from RENDPARAMS;
    //    ExtraLight modifies it via either an additive bias or a hard
    //    override (which also forces ambient-only mode — no contribution
    //    from cell lights).
    Vector3 ambient = mRp->ambientLight;
    bool ambientOnly = false;
    if (mPropSvc) {
        PropExtraLight el{};
        // Property name is the pldef property name ("ExtraLigh" — 8 chars,
        // truncated), NOT the friendly label "ExtraLight". The schema
        // loader registers under the pldef name.
        if (getTypedProperty<PropExtraLight>(mPropSvc, "ExtraLigh", objId, el)) {
            // Original engine: ambient[c] += factor; if (ambient + hilight > 1)
            // ambient = 1 - hilight. Frob highlight is added in the shader
            // (separate uniform), so we just clamp the high side at 1.0.
            // Negative factors are intentional (darkening); do NOT clamp at 0.
            for (int c = 0; c < 3; ++c) {
                ambient[c] += el.factor;
                if (ambient[c] > 1.0f) ambient[c] = 1.0f;
            }
            if (el.isAdditive == 0)
                ambientOnly = true;
        }
    }

    Vector3 total = ambient;
    if (ambientOnly) return total;
    if (cell < 0 || cell >= static_cast<int32_t>(mWr->numCells)) return total;

    const auto &lt = mWr->cells[cell].lightIndices;
    if (lt.empty()) return total;
    int n = static_cast<int>(lt[0]);
    if (n <= 0) return total;

    // 3. Patch sun slot for this object's position. The on-disk slot 0
    //    contains scratch data; we always overwrite before evaluating. The
    //    sun is conceptually placed in the object's "up" direction (along
    //    the inverted sunlight vector) at a fixed distance plus the object
    //    radius, so the 1/r falloff yields a brightness near sunScaledRgb /
    //    SUNLIGHT_DISTANCE.
    mSunSlot.loc = pos + mRp->sunlightNorm * (kSunlightDistance + radius);
    mSunSlot.dir = Vector3(0.0f);
    mSunSlot.bright = mRp->sunScaledRgb;
    mSunSlot.inner = -1.0f;
    mSunSlot.outer = 0.0f;
    mSunSlot.radius = 0.0f;

    // 4. Refresh shadow cache when the cell changes or the object has moved
    //    further than the polling epsilon. This is the same trigger the
    //    original engine uses, just polled here rather than driven by
    //    position-change listeners.
    auto &cache = mCache[objId];
    bool needRebuild = !cache.valid || cache.cellId != cell;
    if (!needRebuild) {
        Vector3 d = pos - cache.pos;
        if (glm::dot(d, d) > kShadowCacheRepollEps * kShadowCacheRepollEps)
            needRebuild = true;
    }

    if (needRebuild) {
        cache.bits[0] = cache.bits[1] = cache.bits[2] = 0;
        // Limit to 96 bits (3 × uint32) — matches engine's per-object shadow
        // storage. Cells with more than 96 lights would silently lose the
        // tail; in practice this never happens (T2 missions cap below 96).
        int evalN = std::min(n, 96);
        for (int i = 0; i < evalN; ++i) {
            int idx = lt[1 + i];
            if (locationSeesLight(pos, cell, idx)) {
                cache.bits[i >> 5] |= (1u << (i & 31));
            }
        }
        cache.cellId = cell;
        cache.pos = pos;
        cache.valid = true;
    }

    // 5. Sum contributions from visible static lights.
    int evalN = std::min(n, 96);
    for (int i = 0; i < evalN; ++i) {
        if (cache.bits[i >> 5] & (1u << (i & 31))) {
            applyOneLight(lt[1 + i], pos, total);
        }
    }

    // 6. Dynamic lights — gameplay-driven transient sources (player
    //    flashlight, fire arrows, mage spells, creature glow). No
    //    per-object visibility cache: the lights move every frame, so we
    //    raycast fresh each call. Cost is O(numDynamicLights × visibleObjects)
    //    per frame; with the engine's cap of 32 simultaneous dynamic lights
    //    and typical visible-object counts in the low hundreds, this stays
    //    cheap.
    if (mDyn) {
        for (int i = 0; i < mDyn->count(); ++i) {
            const DynamicLight &dl = (*mDyn)[i];
            // Radius cutoff first (skip the raycast entirely if out of range).
            if (dl.radius > 0.0f) {
                Vector3 d = pos - dl.loc;
                if (glm::dot(d, d) > dl.radius * dl.radius) continue;
            }
            // Portal raycast — same visibility test the engine performs for
            // dynamic lights at runtime. Cell hint accelerates traversal.
            RayHit hit;
            int32_t terminalCell = -1;
            bool blocked = raycastWorld(*mWr, pos, dl.loc, hit, &terminalCell, cell);
            if (blocked) continue;
            // 1/r distance attenuation, applied per-channel.
            Vector3 d = dl.loc - pos;
            float mag = glm::length(d);
            if (mag < 1e-6f) {
                total += dl.bright;
            } else {
                total += dl.bright * (1.0f / mag);
            }
        }
    }

    // Clamp per channel at 1.0 so stacked bright contributions can't drive
    // dim textures past their natural color when multiplied in the shader.
    // This mirrors the original engine's CLUT path which clamps brightness
    // at 0.99 before indexing the palette table; modern GPUs would clamp at
    // the framebuffer stage anyway, but doing it here keeps dim textures
    // from washing out toward white when bright lights sum past 1.0.
    if (total.x > 1.0f) total.x = 1.0f;
    if (total.y > 1.0f) total.y = 1.0f;
    if (total.z > 1.0f) total.z = 1.0f;
    if (total.x < 0.0f) total.x = 0.0f;
    if (total.y < 0.0f) total.y = 0.0f;
    if (total.z < 0.0f) total.z = 0.0f;

    return total;
}

inline void ObjectIlluminator::buildLightArray(int32_t objId,
                                                const Vector3 &pos,
                                                float radius,
                                                int32_t cellHint,
                                                GPULightArray &out) {
    out.count = 0;
    out.ambient = Vector3(0.0f);
    if (!mWr || !mRp) return;

    // Steps 1-2 mirror `compute()`: resolve cell, ambient + ExtraLight.
    int32_t cell = (cellHint >= 0 && cellHint < static_cast<int32_t>(mWr->numCells))
                 ? cellHint
                 : findCameraCell(*mWr, pos.x, pos.y, pos.z);

    Vector3 ambient = mRp->ambientLight;
    bool ambientOnly = false;
    if (mPropSvc) {
        PropExtraLight el{};
        if (getTypedProperty<PropExtraLight>(mPropSvc, "ExtraLigh", objId, el)) {
            for (int c = 0; c < 3; ++c) {
                ambient[c] += el.factor;
                if (ambient[c] > 1.0f) ambient[c] = 1.0f;
            }
            if (el.isAdditive == 0)
                ambientOnly = true;
        }
    }
    out.ambient = ambient;

    if (ambientOnly) return;
    if (cell < 0 || cell >= static_cast<int32_t>(mWr->numCells)) return;

    const auto &lt = mWr->cells[cell].lightIndices;
    if (lt.empty()) return;
    int n = static_cast<int>(lt[0]);
    if (n <= 0) return;

    // Step 3: patch the sun slot for this object's position. Same as
    // `compute()`. The sun is conceptually placed `kSunlightDistance +
    // radius` units along the inverted sunlight vector; the per-vertex
    // shader will see this as a near-directional light because that
    // distance is large compared to typical object size.
    mSunSlot.loc = pos + mRp->sunlightNorm * (kSunlightDistance + radius);
    mSunSlot.dir = Vector3(0.0f);
    mSunSlot.bright = mRp->sunScaledRgb;
    mSunSlot.inner = -1.0f;
    mSunSlot.outer = 0.0f;
    mSunSlot.radius = 0.0f;

    // Step 4: shadow cache refresh. Identical logic to `compute()` —
    // both lighting paths share the same per-object visibility set.
    auto &cache = mCache[objId];
    bool needRebuild = !cache.valid || cache.cellId != cell;
    if (!needRebuild) {
        Vector3 d = pos - cache.pos;
        if (glm::dot(d, d) > kShadowCacheRepollEps * kShadowCacheRepollEps)
            needRebuild = true;
    }
    if (needRebuild) {
        cache.bits[0] = cache.bits[1] = cache.bits[2] = 0;
        int evalN = std::min(n, 96);
        for (int i = 0; i < evalN; ++i) {
            int idx = lt[1 + i];
            if (locationSeesLight(pos, cell, idx)) {
                cache.bits[i >> 5] |= (1u << (i & 31));
            }
        }
        cache.cellId = cell;
        cache.pos = pos;
        cache.valid = true;
    }

    // Step 5: pack visible static lights into `out.{loc,dir,bright}[i]`.
    // Unlike `compute()` we skip the cosine and the 1/dist factors —
    // the GPU applies both per-vertex. The animated-light multiplier
    // IS folded here (the shader doesn't know about animation).
    int evalN = std::min(n, kObjectLightCap);
    for (int i = 0; i < evalN && out.count < kObjectLightCap; ++i) {
        if (!(cache.bits[i >> 5] & (1u << (i & 31)))) continue;

        int32_t lightIdx = lt[1 + i];
        const WRStaticLight *L = nullptr;
        float multiplier = 1.0f;
        if (lightIdx == 0) {
            L = &mSunSlot;
        } else {
            if (lightIdx < 0 ||
                lightIdx >= static_cast<int32_t>(mWr->staticLights.size()))
                continue;
            L = &mWr->staticLights[lightIdx];
            if (lightIdx < static_cast<int32_t>(mLightMultiplier.size()))
                multiplier = mLightMultiplier[lightIdx];
            if (multiplier <= 0.0f) continue;
        }

        int slot = out.count++;
        out.loc[slot]    = glm::vec4(L->loc, L->inner);
        out.dir[slot]    = glm::vec4(L->dir, L->outer);
        out.bright[slot] = glm::vec4(L->bright * multiplier,
                                     L->radius > 0.0f ? L->radius * L->radius : 0.0f);
    }

    // Step 6: dynamic lights — same portal raycast as `compute()`,
    // packed as omni (loc.w = -1, dir = 0). Dropped silently if the
    // array is already at the cap; in shipping content the static
    // count is far below the cap so this is a paper edge case.
    if (mDyn) {
        for (int i = 0; i < mDyn->count() && out.count < kObjectLightCap; ++i) {
            const DynamicLight &dl = (*mDyn)[i];
            if (dl.radius > 0.0f) {
                Vector3 d = pos - dl.loc;
                if (glm::dot(d, d) > dl.radius * dl.radius) continue;
            }
            RayHit hit;
            int32_t terminalCell = -1;
            bool blocked = raycastWorld(*mWr, pos, dl.loc, hit,
                                        &terminalCell, cell);
            if (blocked) continue;

            int slot = out.count++;
            out.loc[slot]    = glm::vec4(dl.loc, -1.0f);
            out.dir[slot]    = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
            out.bright[slot] = glm::vec4(dl.bright,
                                         dl.radius > 0.0f ? dl.radius * dl.radius : 0.0f);
        }
    }
}

inline bool ObjectIlluminator::locationSeesLight(const Vector3 &from,
                                                 int32_t cellHint,
                                                 int32_t lightIdx) const {
    if (lightIdx == 0) {
        // Sun visibility: at level bake the sun was added to `light_indices`
        // for every cell with sky access. Trust that filter — a runtime ray
        // toward the sun's virtual position would just confirm what the
        // baker already determined and burns CPU on every cache miss.
        return true;
    }
    if (!mWr) return false;
    if (lightIdx < 0 || lightIdx >= static_cast<int32_t>(mWr->staticLights.size()))
        return false;

    const Vector3 &lightLoc = mWr->staticLights[lightIdx].loc;

    RayHit hit;
    int32_t terminalCell = -1;
    bool hitWorld = raycastWorld(*mWr, from, lightLoc, hit, &terminalCell, cellHint);
    return !hitWorld;
}

inline void ObjectIlluminator::applyOneLight(int32_t lightIdx,
                                              const Vector3 &pos,
                                              Vector3 &total) const {
    if (!mWr) return;

    // Slot 0: take the patched sun copy that compute() wrote earlier.
    const WRStaticLight *L = nullptr;
    float multiplier = 1.0f;
    if (lightIdx == 0) {
        L = &mSunSlot;
        // Sun slot: no animated multiplier (sun is constant).
    } else {
        if (lightIdx < 0 || lightIdx >= static_cast<int32_t>(mWr->staticLights.size()))
            return;
        L = &mWr->staticLights[lightIdx];
        // Per-light brightness scale from the animated-light system: a
        // flickering torch reduces its multiplier toward zero, dimming any
        // object close enough to be lit by it. Multiplier of 0 means the
        // light is fully off → skip work entirely.
        if (lightIdx < static_cast<int32_t>(mLightMultiplier.size()))
            multiplier = mLightMultiplier[lightIdx];
        // Diagnostic: log the first time each slot is read with a dimmed
        // multiplier. Per-slot dedup so a permanently-off slot (e.g. one
        // baked at zero brightness) doesn't drown out real toggle events.
        if (multiplier < 0.99f) {
            static std::unordered_set<int32_t> sSeen;
            if (sSeen.insert(lightIdx).second)
                std::fprintf(stderr,
                    "[OBJ-LIGHT] applyOneLight slot=%d mult=%.2f "
                    "bright=(%.2f,%.2f,%.2f)\n",
                    lightIdx, multiplier,
                    L->bright.x, L->bright.y, L->bright.z);
        }
        if (multiplier <= 0.0f) return;
    }

    // Radius cutoff (skip lights that are out of range for this point).
    if (lightIdx != 0 && L->radius > 0.0f) {
        Vector3 d = pos - L->loc;
        float dist2 = glm::dot(d, d);
        if (dist2 > L->radius * L->radius) return;
    }

    // Spotlight cone: linear interpolation between cos(inner) (full bright)
    // and cos(outer) (zero). inner == -1.0 sentinel means omni-directional.
    float scale = 1.0f;
    if (L->inner != -1.0f) {
        Vector3 d = pos - L->loc;
        float dlen = glm::length(d);
        if (dlen < 1e-6f) return;
        d /= dlen;
        float dotVal = glm::dot(d, L->dir);
        if (dotVal >= L->inner) {
            scale = 1.0f;
        } else if (dotVal <= L->outer) {
            scale = 0.0f;
        } else {
            float denom = (L->inner - L->outer);
            scale = (denom > 1e-6f) ? (dotVal - L->outer) / denom : 0.0f;
        }
        if (scale <= 0.0f) return;
    }

    // 1/r distance attenuation, applied per RGB channel. Matches the
    // engine's CLUT-path attenuation. The animated-light multiplier scales
    // the entire contribution from this slot.
    Vector3 d = L->loc - pos;
    float mag = glm::length(d);
    if (mag < 1e-6f) {
        // Coincident light and object — clamp to avoid divide-by-zero. The
        // contribution would be huge anyway; cap at brightness to keep the
        // result finite.
        total += L->bright * (scale * multiplier);
        return;
    }
    total += L->bright * (scale * multiplier / mag);
}

} // namespace Darkness
