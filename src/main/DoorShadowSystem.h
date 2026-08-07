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

// S3 — doors casting shadows (PLAN.HIGH_RES_SHADOWS §S3 design).
//
// Door-adjacent lights are OVERLAY lights: the bake excludes them from the
// base and gives each its own per-poly overlay, exactly like an animated
// light — so a door event re-bakes ONE light's overlays over its reach
// polys (bounded) instead of the base's all-lights sum, and the existing
// blend + per-poly upload machinery does the rest
// (`blendRebakedLightmap` already defaults overlays without an intensity
// entry to 1.0, which is precisely what a geometry-only overlay wants).
//
// This header owns: the door census (which doors, their conservative swept
// spheres), the door-adjacent light selection, the bake-ray occlusion hook
// (door OBBs only, applied only to rays of door-adjacent lights — by
// construction no other light's rays can cross a door within its reach),
// and the runtime event → budgeted overlay re-bake → blend handoff.
//
// Vintage stays exact: everything here acts on the re-baked path only.

#pragma once

#include "LightmapBake.h"
#include "LumelBakeGPU.h"
#include "../services/physics/ObjectCollisionGeometry.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Darkness {

// One door the shadow system tracks. `sweptRadius` is a conservative
// sphere over the leaf's whole travel: hinge-anchored position (see
// HANDOFF.DOOR_PORTAL_CULLING — a swung leaf ends a door's width from its
// placement) plus the longest leaf edge, with slack. Conservative is
// correct here: it only widens the "which lights can this door affect"
// selection, never the shadows themselves.
struct DoorShadowDoor {
    int32_t  objID = 0;
    uint32_t bodyIdx = 0;    // index into ObjectCollisionWorld bodies
    Vector3  pos{0.0f};
    float    sweptRadius = 0.0f;
};

// The bake-ray hook: is this WR-clear segment blocked by a door leaf?
// Door counts are small (tens), so a flat walk with the exact test is
// affordable — and the hook only ever runs for door-adjacent lights'
// rays. Thread-safe: read-only over ObjectCollisionWorld while the load
// bake's workers run (the main thread is blocked in the bake).
struct DoorSegmentOcclusion {
    const ObjectCollisionWorld *world = nullptr;
    std::vector<uint32_t> bodies;

    static bool blocked(const void *ctxRaw, const Vector3 &a,
                        const Vector3 &b) {
        const auto *ctx = static_cast<const DoorSegmentOcclusion *>(ctxRaw);
        if (!ctx || !ctx->world) return false;
        for (uint32_t bi : ctx->bodies)
            if (ctx->world->segmentHitsBody(a, b, bi)) return true;
        return false;
    }
};

// Census: every collision body that DoorSystem claims as a door. `isDoor`
// arrives as a predicate so this header does not include the sim stack.
inline std::vector<DoorShadowDoor> buildDoorShadowCensus(
    const ObjectCollisionWorld &ocw,
    const std::function<bool(int32_t)> &isDoor) {
    std::vector<DoorShadowDoor> out;
    const auto &bodies = ocw.getBodies();
    for (size_t i = 0; i < bodies.size(); ++i) {
        const ObjectCollisionBody &b = bodies[i];
        if (b.removed || !isDoor(b.objID)) continue;
        const float maxEdge =
            std::max({b.edgeLengths.x, b.edgeLengths.y, b.edgeLengths.z});
        if (maxEdge <= 0.0f) continue;   // no usable leaf box
        DoorShadowDoor d;
        d.objID = b.objID;
        d.bodyIdx = static_cast<uint32_t>(i);
        d.pos = b.worldPos;
        // 1.1x the longest leaf edge: the swing sphere of a hinge-anchored
        // leaf (1.5x measured 2.25x the cone AREA and inflated every event's
        // work list for nothing).
        d.sweptRadius = 1.1f * maxEdge;
        out.push_back(d);
    }
    return out;
}

// Door-adjacent light selection: reach sphere vs swept sphere. `reach`
// is ShadowMapCache::lightReach / the bake's reachRadii — the ONE reach
// authority (invariant #8).
inline std::vector<uint8_t> doorAdjacentLights(
    const WRParsedData &wr, const std::vector<float> &reach,
    const std::vector<DoorShadowDoor> &doors) {
    std::vector<uint8_t> flags(wr.staticLights.size(), 0);
    for (size_t li = 1; li < wr.staticLights.size(); ++li) {
        if (li >= reach.size() || reach[li] <= 0.0f) continue;
        const Vector3 &lp = wr.staticLights[li].loc;
        for (const DoorShadowDoor &d : doors) {
            if (glm::length(d.pos - lp) <= reach[li] + d.sweptRadius) {
                flags[li] = 1;
                break;
            }
        }
    }
    return flags;
}

// ── The runtime system ─────────────────────────────────────────────────────
// Owned by main() beside DoorSystem. Flow per door event:
//   DoorSystem collision callback → markDirty(objID)
//   update() (frame loop): rate-limited work-list rebuild from the dirty
//     set, then a per-frame poly budget of single-light overlay re-bakes
//     at the doors' CURRENT poses; finished recs go to `readyRecs`, which
//     updateLightmaps consumes to re-blend + upload via the existing path
//     (visibility-filter bypassed — a door event is an envelope event).
class DoorShadowSystem {
public:
    // Everything the runtime re-bake needs, captured at load-bake time so
    // event re-bakes use EXACTLY the load formula (same anchors, same
    // penumbra, same hook). `animPolys` is gpu.rebakedAnimPolys.
    void init(const WRParsedData *wr, const RenderParams *rp,
              const BakeFormula &overlayFormula,
              std::vector<float> anchors,
              std::vector<int32_t> lightCells,
              std::vector<DoorShadowDoor> doors,
              std::vector<uint8_t> extraLights,
              const ObjectCollisionWorld *ocw,
              std::vector<RebakedAnimPoly> *animPolys,
              AtlasTexture *dirAtlas = nullptr,
              const LightmapAtlasSet *atlasSet = nullptr,
              int density = 1) {
        mWr = wr;
        mRp = rp;
        mFormula = overlayFormula;
        mAnchors = std::move(anchors);
        mLightCells = std::move(lightCells);
        mDoors = std::move(doors);
        mExtraLights = std::move(extraLights);
        mAnimPolys = animPolys;
        mDirAtlas = dirAtlas;
        mAtlasSet = atlasSet;
        mDensity = density;
        // Per-light reach cache for the direction pass's candidate
        // gathering (all lights, not just the event's).
        mReach.assign(mWr ? mWr->staticLights.size() : 0, 0.0f);
        if (mWr)
            for (size_t li = 1; li < mWr->staticLights.size(); ++li)
                mReach[li] = physicalReachRadius(
                    mWr->staticLights[li],
                    li < mAnchors.size() ? mAnchors[li]
                                         : mFormula.falloffAnchor,
                    mFormula.emitterRadius, mFormula.brightScale,
                    /*clampToAuthored=*/mFormula.throwAlpha <= 0.0f);
        // Re-bind the formula's non-owning pointers to OUR storage — the
        // bake-time vectors died with the bake. EVERY pointer field must
        // be re-bound here or the runtime re-bake reads freed memory.
        mFormula.perLightAnchor =
            mAnchors.empty() ? nullptr : &mAnchors;
        mFormula.extraOverlayLights =
            mExtraLights.empty() ? nullptr : &mExtraLights;
        mOcclusion.world = ocw;
        mOcclusion.bodies.clear();
        for (const DoorShadowDoor &d : mDoors)
            mOcclusion.bodies.push_back(d.bodyIdx);
        mFormula.segmentBlockedFn = &DoorSegmentOcclusion::blocked;
        mFormula.segmentBlockedCtx = &mOcclusion;
        // Overlay bakes carry one light, nothing else — mirror the load
        // driver's overlayF construction.
        mFormula.includeAmbient = false;
        mFormula.includeSun = false;
        // Runtime re-bakes trade emitter samples for latency. At load
        // quality (16-sample penumbra x 2x2 receiver supersampling) ONE
        // door's event measured 8.5M rays / 4.3 s of work — an ~18 s
        // trickle at the frame budget, which reads as "the framerate
        // never recovered". The fast profile (4 x 1x1) cuts ~16x; door
        // shadows during/after a swing are slightly harder-edged than
        // load-baked ones. A quality re-pass once the door RESTS is the
        // recorded follow-up if the seam shows.
        mFormula.penumbraSamples = std::min(mFormula.penumbraSamples, 4);
        mFormula.supersample = 1;

        // light → rec indices whose overlays include it.
        mLightRecs.clear();
        if (mAnimPolys) {
            for (size_t r = 0; r < mAnimPolys->size(); ++r)
                for (int16_t li : (*mAnimPolys)[r].overlayLightIdx)
                    if (li > 0) mLightRecs[li].push_back(r);
        }
        // Per-rec poly bounding spheres for the shadow-cone work-list
        // filter: an event re-bakes a (rec, light) pair only if the
        // segment poly->light crosses THE dirty door's swept sphere.
        // Without this the list is the light's whole mission-wide rec set
        // (measured: never drains under a swinging door's rebuild rate).
        mRecSphere.clear();
        if (mAnimPolys && mWr) {
            mRecSphere.resize(mAnimPolys->size(), glm::vec4(0.0f));
            for (size_t r = 0; r < mAnimPolys->size(); ++r) {
                const RebakedAnimPoly &rec = (*mAnimPolys)[r];
                if (rec.ci >= mWr->numCells) continue;
                const WRParsedCell &cell = mWr->cells[rec.ci];
                if (rec.pi < 0 ||
                    rec.pi >= static_cast<int>(cell.polyIndices.size()))
                    continue;
                const auto &pidx = cell.polyIndices[rec.pi];
                if (pidx.empty()) continue;
                Vector3 pc(0.0f);
                for (uint8_t vi : pidx) pc += cell.vertices[vi];
                pc /= static_cast<float>(pidx.size());
                float pr = 0.0f;
                for (uint8_t vi : pidx)
                    pr = std::max(pr, glm::length(cell.vertices[vi] - pc));
                mRecSphere[r] = glm::vec4(pc.x, pc.y, pc.z, pr);
            }
        }

        // Per-door STATIC work lists: the shadow-cone test runs against
        // the door's SWEPT sphere, which never moves — so each door's
        // (rec, light) set is computable once, here, pre-sorted
        // nearest-first. The first cut recomputed this scan on every
        // 0.15 s rebuild; now a rebuild just splices lists.
        mDoorWork.clear();
        size_t staticWorkTotal = 0;
        for (const DoorShadowDoor &d : mDoors) {
            auto &work = mDoorWork[d.objID];
            for (const auto &[li, recs] : mLightRecs) {
                if (static_cast<size_t>(li) >= mExtraLights.size() ||
                    !mExtraLights[li])
                    continue;
                const Vector3 lp = mWr->staticLights[li].loc;
                for (size_t r : recs) {
                    if (r >= mRecSphere.size()) continue;
                    const glm::vec4 &ps = mRecSphere[r];
                    if (segmentNearSphere(Vector3(ps.x, ps.y, ps.z), lp,
                                          d.pos, d.sweptRadius + ps.w))
                        work.push_back({r, li});
                }
            }
            std::sort(work.begin(), work.end(),
                [&](const std::pair<size_t, int16_t> &a,
                    const std::pair<size_t, int16_t> &b) {
                    auto dist = [&](size_t r) {
                        const glm::vec4 &ps = mRecSphere[r];
                        return glm::length(
                            Vector3(ps.x, ps.y, ps.z) - d.pos);
                    };
                    return dist(a.first) < dist(b.first);
                });
            staticWorkTotal += work.size();
        }

        // Byte census: what a K-pose pre-bake would multiply.
        size_t doorOverlayBytes = 0, doorOverlayCount = 0;
        if (mAnimPolys) {
            for (const auto &rec : *mAnimPolys)
                for (size_t k = 0; k < rec.overlayLightIdx.size(); ++k) {
                    const int16_t li = rec.overlayLightIdx[k];
                    if (li > 0 &&
                        static_cast<size_t>(li) < mExtraLights.size() &&
                        mExtraLights[li] &&
                        k < rec.overlays.size()) {
                        doorOverlayBytes += rec.overlays[k].size();
                        ++doorOverlayCount;
                    }
                }
        }
        std::fprintf(stderr,
            "Door shadows: %zu door-adjacent overlay buffer(s), %.1f MB "
            "(the base a K-pose pre-bake multiplies)\n",
            doorOverlayCount, doorOverlayBytes / 1048576.0);
        // Name the most light-coupled doors — the doors whose swings
        // actually exercise this system. `--stress-doors` picks by
        // camera distance, and a spawn-area door with no lights near it
        // stress-tests NOTHING; pin one of these with
        // `--stress-door-ids <id>` instead.
        {
            std::vector<std::pair<size_t, int32_t>> rank;
            for (const auto &dw : mDoorWork)
                if (!dw.second.empty())
                    rank.push_back({dw.second.size(), dw.first});
            std::sort(rank.rbegin(), rank.rend());
            std::string line;
            char buf[64];
            for (size_t i = 0; i < rank.size() && i < 5; ++i) {
                std::snprintf(buf, sizeof(buf), "%s%d (%zu items)",
                              i ? ", " : "", rank[i].second,
                              rank[i].first);
                line += buf;
            }
            std::fprintf(stderr,
                "[DOOR_SHADOW] most light-coupled doors: %s\n",
                line.empty() ? "none" : line.c_str());
        }
        // ── Init-time direction recombine ──
        // Door-adjacent lights contribute no direction at LOAD (or the
        // runtime recombine would double-count them) — paint their term
        // in now through the SAME code path door events use: one path,
        // consistency by construction. The idempotency tripwire re-runs
        // a sample and demands bit-identical texels (the recombine is
        // deterministic; any diff means a real bug).
        if (mDirAtlas && !mDirAtlas->rgba.empty() && mAnimPolys) {
            const auto t0 = std::chrono::steady_clock::now();
            size_t painted = 0;
            for (size_t r = 0; r < mAnimPolys->size(); ++r)
                if (!(*mAnimPolys)[r].dirBase.empty()) {
                    recombineDirection(r);
                    ++painted;
                }
            const std::vector<uint8_t> snap = mDirAtlas->rgba;
            size_t retested = 0;
            for (size_t r = 0;
                 r < mAnimPolys->size() && retested < 100; ++r)
                if (!(*mAnimPolys)[r].dirBase.empty()) {
                    recombineDirection(r);
                    ++retested;
                }
            const bool idempotent = (snap == mDirAtlas->rgba);
            std::fprintf(stderr,
                "[DIR_RECOMBINE] init: %zu door polys painted in %.0f ms; "
                "idempotency over %zu re-runs: %s\n",
                painted,
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count(),
                retested, idempotent ? "PASS" : "FAIL");
            // The renderer uploads the whole dir texture once after init;
            // per-rect dirty tracking starts clean.
            mDirDirty.clear();
            mDirInitPainted = true;
        }

        std::fprintf(stderr,
            "Door shadows: %zu door(s), %d door-adjacent light(s), "
            "%zu light->poly overlay lists\n",
            mDoors.size(),
            static_cast<int>(std::count(mExtraLights.begin(),
                                        mExtraLights.end(), 1)),
            mLightRecs.size());
    }

    bool active() const { return mAnimPolys && !mDoors.empty(); }

    // From the DoorSystem collision-update callback (per animation tick).
    // The matrix gate makes this robust against any at-rest callback: a
    // pose that has not actually moved never dirties anything.
    void markDirty(int32_t objID, const Matrix4 &worldMatrix) {
        if (!active()) return;
        for (const DoorShadowDoor &d : mDoors)
            if (d.objID == objID) {
                auto it = mLastPose.find(objID);
                if (it != mLastPose.end()) {
                    const Matrix4 &prev = it->second;
                    float maxd = 0.0f;
                    for (int c = 0; c < 4; ++c)
                        for (int r = 0; r < 4; ++r)
                            maxd = std::max(maxd,
                                std::abs(prev[c][r] - worldMatrix[c][r]));
                    if (maxd < 1e-4f) return;   // pose unchanged
                }
                mLastPose[objID] = worldMatrix;
                mDirtyDoors.insert(objID);
                // S4c: a real pose change marks the door IN MOTION —
                // update() promotes its lights to differential live and
                // defers the re-bake event until the door settles.
                mLastMotion[objID] = std::chrono::steady_clock::now();
                return;
            }
    }

    // Rec indices whose overlays were re-baked this frame; consumed (and
    // cleared) by updateLightmaps, which re-blends them with the
    // visibility filter bypassed.
    std::vector<size_t> &readyRecs() { return mReadyRecs; }

    // Recs whose DIRECTION texels were re-encoded this frame — the
    // renderer uploads their rects from the CPU dir atlas.
    std::vector<size_t> &dirDirtyRecs() { return mDirDirty; }

    // S4c: lights currently promoted to DIFFERENTIAL live rendering
    // because a door is swinging through their cone. The frame loop
    // appends these to the live-light list; the shader adds
    // K·falloff·cos·(shadow(slotCurrent) − shadow(slotFrozen)) — the
    // signed correction over the still-untouched baked overlay. Rebuilt
    // every update().
    struct PromotedLive {
        Vector3 pos{0.0f};
        Vector3 colorK{0.0f};
        float   reach2 = 0.0f;
        int     slotFrozen = -1;
        int     slotCurrent = -1;
    };
    const std::vector<PromotedLive> &promotedLive() const {
        return mPromotedLive;
    }

    // The doors currently dirty or being processed — the object-lighting
    // invalidation radius comes from here (DarknessRender.cpp couples it
    // to ObjectIlluminator::invalidateNear).
    const std::vector<DoorShadowDoor> &doors() const { return mDoors; }

    // For ObjectIlluminator's dynamic-occlusion hook (same fn+ctx shape
    // the bake uses — ObjectIllumination.h cannot include this header,
    // the include chain runs the other way).
    const DoorSegmentOcclusion *occlusion() const { return &mOcclusion; }

    // Frame update: rebuild the work list at most every kRebuildSec while
    // doors are dirty (a swinging door re-dirties every tick — the rate
    // limit coalesces; the final at-rest pose is picked up by the last
    // rebuild), then burn the per-frame poly budget.
    // Returns (center, radius) invalidation spheres for this rebuild's
    // doors — radius spans the door's swept sphere plus the largest
    // affected light's reach, so every object whose sight-line to an
    // affected light could cross the door re-runs its visibility rays.
    // gpu/sc may be null (headless / --rebake-cpu-events): the CPU ray
    // path then burns the budget exactly as before.
    std::vector<std::pair<Vector3, float>> update(
        LumelBakeGPU *gpu = nullptr, ShadowMapCache *sc = nullptr,
        uint32_t bgfxFrame = 0, bool cpuEvents = false) {
        std::vector<std::pair<Vector3, float>> rebuiltFor;
        if (!active()) return rebuiltFor;
        const auto now = std::chrono::steady_clock::now();
        const bool gpuMode = !cpuEvents && gpu && gpu->valid() && sc &&
                             sc->valid();

        // ── Collect a landed GPU batch ──
        if (gpuMode && gpu->batch.inFlight &&
            bgfxFrame >= gpu->batch.readyFrame) {
            const bool flip = bgfx::getCaps()->originBottomLeft;
            const int S = LumelBakeGPU::kScratchSize;
            for (const auto &it : gpu->batch.items) {
                if (it.recIdx >= mAnimPolys->size()) continue;
                RebakedAnimPoly &rec = (*mAnimPolys)[it.recIdx];
                if (it.ovK >= rec.overlays.size()) continue;
                std::vector<uint8_t> &buf = rec.overlays[it.ovK];
                if (static_cast<int>(buf.size()) < it.w * it.h * 3)
                    buf.assign(static_cast<size_t>(it.w) * it.h * 3, 0);
                for (int y = 0; y < it.h; ++y) {
                    const int sy = it.y + y;
                    const int row = flip ? (S - 1 - sy) : sy;
                    const uint8_t *src =
                        &gpu->batch.pixels[(static_cast<size_t>(row) * S +
                                            it.x) * 4];
                    uint8_t *dst =
                        &buf[static_cast<size_t>(y) * it.w * 3];
                    for (int x = 0; x < it.w; ++x) {
                        dst[x * 3 + 0] = src[x * 4 + 0];
                        dst[x * 3 + 1] = src[x * 4 + 1];
                        dst[x * 3 + 2] = src[x * 4 + 2];
                    }
                }
                mReadyRecs.push_back(it.recIdx);
                // Direction: queue the ray-free RECOMBINE (slice 3) —
                // budgeted per frame below, ungated (every door poly's
                // direction tracks door state; the dim-but-directional
                // polys are exactly where specular-through-doors lives).
                if (mEventDirDone.insert(it.recIdx).second)
                    mDirQueue.push_back(it.recIdx);
            }
            mEventGpuRects += gpu->batch.items.size();
            gpu->batch.items.clear();
            gpu->batch.inFlight = false;
        }

        // ── S4c: differential live promotion while doors move ──
        // The re-bake event is DEFERRED until every dirty door has
        // settled (no pose change for kSettleSec): while a leaf is
        // moving, the promoted lights' differential term carries the
        // shadow per-pixel with zero lag, so mid-swing re-bakes are
        // pure waste — they were also the cost that made door opening
        // slow. One event fires at the final pose.
        const bool liveOk = sc && sc->valid() && mWr;
        if (liveOk) {
            if (!mPoseSnapInit) {
                mPoseSnapInit = true;
                snapshotBakedPoses(nullptr);
            }
            for (const auto &lm : mLastMotion)
                if (std::chrono::duration<float>(now - lm.second).count() <
                    kSettleSec)
                    promoteDoorLights(*sc, lm.first, bgfxFrame, now,
                                      rebuiltFor);
            // Per-frame upkeep: the current-pose transient re-renders
            // exactly when the door pose changed (caster hash); the
            // frozen slot only gets its LRU stamp.
            mPromotedLive.clear();
            for (PromotedLight &p : mPromoted) {
                const size_t li = static_cast<size_t>(p.lightIdx);
                if (li >= mWr->staticLights.size() ||
                    li >= mReach.size()) continue;
                const WRStaticLight &L = mWr->staticLights[li];
                const float reach = mReach[li];
                if (reach <= 0.0f) continue;
                p.slotCurrent = ensureDynamicShadowLight(
                    *sc, *mWr, p.dynId, L.loc, reach, bgfxFrame);
                touchShadowSlot(*sc, p.slotFrozen, bgfxFrame);
                if (p.slotFrozen < 0 || p.slotCurrent < 0) continue;
                PromotedLive pl;
                pl.pos = L.loc;
                pl.colorK = L.bright *
                            (mFormula.brightScale * promoteK(p.lightIdx));
                pl.reach2 = reach * reach;
                pl.slotFrozen = p.slotFrozen;
                pl.slotCurrent = p.slotCurrent;
                mPromotedLive.push_back(pl);
            }
        }

        // Immediate rebuild when idle (kills the first-frame lag the user
        // reported); rate-limited only while an event is in flight —
        // and settle-gated: no event starts while one of its doors is
        // still moving (see the S4c block above).
        const bool idle = mWork.empty();
        bool allSettled = true;
        for (int32_t objID : mDirtyDoors)
            if (doorMoving(objID, now)) { allSettled = false; break; }
        if (!mDirtyDoors.empty() && allSettled &&
            (idle ||
             std::chrono::duration<float>(now - mLastRebuild).count() >=
                 kRebuildSec)) {
            mLastRebuild = now;
            if (mWorkCursor < mWork.size() && !mWork.empty())
                std::fprintf(stderr,
                    "[DOOR_SHADOW] rebuild preempts unfinished event: "
                    "%zu/%zu polys done (%.1f ms so far)\n",
                    mWorkCursor, mWork.size(), mEventMs);
            // Splice the DIRTY doors' precomputed work lists (each is
            // pose-independent and pre-sorted nearest-first for its door).
            // Invalidation fires at most once per kInvalidateSec per door,
            // radius clamped to door scale: the objects whose lighting a
            // door changes sit near the door — a reach-sized radius
            // (measured up to ~200u under throw intensity) invalidated
            // most of the mission's object caches per rebuild.
            std::unordered_set<uint64_t> seenPair;
            mDirtyDoors.swap(mProcessingDoors);
            mDirtyDoors.clear();
            // S4c: remember which doors this event bakes — the drain
            // refreshes their baked-pose snapshots and demotes their
            // promoted lights.
            mEventDoors = mProcessingDoors;
            mWork.clear();
            mWorkCursor = 0;
            for (int32_t objID : mProcessingDoors) {
                const DoorShadowDoor *d = nullptr;
                for (const auto &dd : mDoors)
                    if (dd.objID == objID) { d = &dd; break; }
                if (!d) continue;
                auto lw = mDoorWork.find(objID);
                if (lw != mDoorWork.end()) {
                    for (const auto &pr : lw->second) {
                        const uint64_t key =
                            (static_cast<uint64_t>(pr.first) << 16) |
                            static_cast<uint16_t>(pr.second);
                        if (seenPair.insert(key).second)
                            mWork.push_back(pr);
                    }
                }
                auto &lastInv = mLastInvalidate[objID];
                if (std::chrono::duration<float>(now - lastInv).count() >=
                    kInvalidateSec) {
                    lastInv = now;
                    rebuiltFor.push_back(
                        {d->pos, d->sweptRadius + kInvalidateMargin});
                }
            }
            mProcessingDoors.clear();
            mEventPolysTotal = mWork.size();
            mEventRays = 0;
            mEventMs = 0.0;
            mEventDirDone.clear();
            mEventGpuRects = 0;
            mEventCpuRects = 0;
            // Rebuild census — the line that distinguishes "no events
            // arrive" from "events arrive but select no work" (both look
            // like a dead feature without it). Cadence-limited by the
            // rebuild rate itself.
            std::fprintf(stderr,
                "[DOOR_SHADOW] rebuild: %zu (poly,light) work items "
                "(%zu door invalidation sphere(s))\n",
                mWork.size(), rebuiltFor.size());
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (gpuMode) {
            // ── Submit phase: pack rects into the scratch, one batch in
            // flight. Spot lights fall back to the CPU path per item (the
            // GPU shader is omni-only — recorded TODO).
            if (!gpu->batch.inFlight && mWorkCursor < mWork.size()) {
                beginLumelBatchView(*gpu);
                int shelfX = 0, shelfY = 0, shelfH = 0;
                const int S = LumelBakeGPU::kScratchSize;
                while (mWorkCursor < mWork.size() &&
                       gpu->batch.items.size() < kGpuRectsPerBatch) {
                    const auto &[recIdx, lightIdx] = mWork[mWorkCursor];
                    const RebakedAnimPoly &rec = (*mAnimPolys)[recIdx];
                    size_t k = 0;
                    for (; k < rec.overlayLightIdx.size(); ++k)
                        if (rec.overlayLightIdx[k] == lightIdx) break;
                    const WRStaticLight &L =
                        mWr->staticLights[static_cast<size_t>(lightIdx)];
                    const bool spot = (L.inner != -1.0f);
                    const LumelGrid grid =
                        buildLumelGrid(*mWr, rec.ci, rec.pi, mDensity);
                    const bool gridOk = grid.valid &&
                                        grid.lx == rec.w &&
                                        grid.ly == rec.h &&
                                        k < rec.overlays.size();
                    if (spot || !gridOk) {
                        // CPU fallback for this item.
                        ++mWorkCursor;
                        rebakeOverlay(recIdx, lightIdx);
                        if (mEventDirDone.insert(recIdx).second)
                            mDirQueue.push_back(recIdx);
                        ++mEventCpuRects;
                        continue;
                    }
                    // Shelf-pack.
                    if (shelfX + grid.lx > S) {
                        shelfY += shelfH;
                        shelfX = 0;
                        shelfH = 0;
                    }
                    if (shelfY + grid.ly > S) break;   // batch full
                    const int slot = ensureShadowLight(
                        *sc, *mWr, lightIdx, bgfxFrame);
                    if (slot < 0) { ++mWorkCursor; continue; }
                    const float anchor = mFormula.anchorFor(lightIdx);
                    const float a2 = mFormula.emitterRadius *
                                     mFormula.emitterRadius;
                    const float K = (anchor * anchor + a2) / anchor;
                    const Vector3 colorK =
                        L.bright * (mFormula.brightScale * K);
                    const float reach =
                        static_cast<size_t>(lightIdx) < mReach.size()
                            ? mReach[static_cast<size_t>(lightIdx)] : 0.0f;
                    if (reach <= 0.0f) { ++mWorkCursor; continue; }
                    if (!submitLumelRect(*gpu, *sc, grid,
                                         mFormula.surfaceOffset, L.loc,
                                         reach, colorK, slot,
                                         mFormula.emitterRadius, shelfX,
                                         shelfY))
                        break;   // transient pool exhausted this frame
                    LumelBakeBatch::Item item;
                    item.recIdx = recIdx;
                    item.ovK = k;
                    item.lightIdx = lightIdx;
                    item.x = shelfX;
                    item.y = shelfY;
                    item.w = grid.lx;
                    item.h = grid.ly;
                    gpu->batch.items.push_back(item);
                    shelfX += grid.lx;
                    shelfH = std::max(shelfH, grid.ly);
                    ++mWorkCursor;
                }
                if (!gpu->batch.items.empty()) {
                    gpu->batch.readyFrame = kickLumelBatchReadback(*gpu);
                    gpu->batch.inFlight = true;
                }
            }
        } else {
            // ── CPU ray path (--rebake-cpu-events / no GPU engine) ──
            int budget = kPolysPerFrame;
            while (budget-- > 0 && mWorkCursor < mWork.size()) {
                const auto &[recIdx, lightIdx] = mWork[mWorkCursor++];
                rebakeOverlay(recIdx, lightIdx);
                if (mEventDirDone.insert(recIdx).second)
                    mDirQueue.push_back(recIdx);
                if (std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count() >
                    kFrameMsBudget)
                    break;
            }
        }
        // ── Direction recombine queue (budgeted; pure math, no rays) ──
        {
            const auto d0 = std::chrono::steady_clock::now();
            while (!mDirQueue.empty()) {
                recombineDirection(mDirQueue.back());
                mDirQueue.pop_back();
                if (std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - d0).count() >
                    kDirMsBudget)
                    break;
            }
        }

        const bool drained =
            mWorkCursor >= mWork.size() &&
            !(gpuMode && gpu->batch.inFlight) && mDirQueue.empty();
        if (mWork.size() && drained) {
            mEventMs += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            // Event complete — report once, rate-limited.
            const auto nowLog = std::chrono::steady_clock::now();
            if (std::chrono::duration<float>(nowLog - mLastLog).count() >=
                1.0f) {
                mLastLog = nowLog;
                std::fprintf(stderr,
                    "[DOOR_SHADOW] event: %zu overlay polys re-baked "
                    "(%zu GPU rects, %zu CPU, %llu CPU rays, %.1f ms CPU "
                    "across frames)\n",
                    mEventPolysTotal, mEventGpuRects, mEventCpuRects,
                    static_cast<unsigned long long>(mEventRays), mEventMs);
            }
            mWork.clear();
            mWorkCursor = 0;
            // S4c: the settle event landed — its doors' overlays now
            // hold the settled pose. Refresh those doors' baked-pose
            // snapshots and let their promotions extinguish (or
            // refreeze, if a door is already moving again). Also fire
            // an UNCONDITIONAL invalidation sphere per event door:
            // motion-time invalidation ran on a cadence, so the
            // settled pose may never have been sampled — and settled
            // is the state objects keep.
            for (int32_t objID : mEventDoors)
                for (const auto &dd : mDoors)
                    if (dd.objID == objID) {
                        mLastInvalidate[objID] = now;
                        rebuiltFor.push_back(
                            {dd.pos,
                             dd.sweptRadius + kInvalidateMargin});
                        break;
                    }
            if (liveOk) settleDrained(*sc, bgfxFrame, now);
            else mEventDoors.clear();
        } else if (mWorkCursor < mWork.size()) {
            mEventMs += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
        }
        return rebuiltFor;
    }

private:
    // Re-bake ONE light's overlay buffer on ONE poly at current door
    // poses, and mark the rec ready for re-blend. The light list handed
    // to bakePolygon is a two-entry "count, light" list — the authored
    // cell list must not gate a physical-mode light (invariant: physical
    // candidates are geometric).
    void rebakeOverlay(size_t recIdx, int16_t lightIdx) {
        if (!mAnimPolys || recIdx >= mAnimPolys->size()) return;
        RebakedAnimPoly &rec = (*mAnimPolys)[recIdx];
        size_t k = 0;
        for (; k < rec.overlayLightIdx.size(); ++k)
            if (rec.overlayLightIdx[k] == lightIdx) break;
        if (k >= rec.overlayLightIdx.size()) return;

        std::vector<uint8_t> mask(mWr->staticLights.size(), 0);
        mask[static_cast<size_t>(lightIdx)] = 1;
        const std::vector<uint16_t> lightList = {
            1, static_cast<uint16_t>(lightIdx)};

        BakeStats stats;
        std::vector<Vector3> lumels;
        if (bakePolygon(*mWr, *mRp, rec.ci, rec.pi, mFormula, mLightCells,
                        lumels, stats, nullptr, mask.data(), nullptr,
                        &lightList) &&
            static_cast<int>(lumels.size()) == rec.w * rec.h) {
            packLumelsRGB8(lumels, rec.overlays[k]);
            mReadyRecs.push_back(recIdx);
        }
        mEventRays += stats.rays;
    }

    // S2 slice 3 — the ray-free direction RECOMBINE. The dir texel is
    //   encode( dirBase(texel) + Σ_doorlights lum(overlay_k, texel) ·
    //           unitDir(light, texel) )
    // dirBase is the load-time snapshot of everything door-static;
    // lum(overlay) is the energy we already re-bake (its visibility IS
    // the door state); unitDir comes from the affine lumel grid. Weight
    // approximation, documented: the load accumulator weighs by LINEAR
    // pre-storage peaks while stored overlays are post-transform
    // (~0.965x linear − toe) — the [DIR_RECOMBINE] identity check at
    // init measures the resulting error. Openness alpha preserved.
    void recombineDirection(size_t recIdx) {
        if (!mDirAtlas || !mAtlasSet || !mAnimPolys ||
            recIdx >= mAnimPolys->size())
            return;
        const RebakedAnimPoly &rec = (*mAnimPolys)[recIdx];
        if (rec.dirBase.size() <
            static_cast<size_t>(rec.w) * rec.h * 4)
            return;                          // not a door-cone poly
        if (rec.ci >= mAtlasSet->entries.size()) return;
        const auto &cellEntries = mAtlasSet->entries[rec.ci];
        if (rec.pi < 0 ||
            rec.pi >= static_cast<int>(cellEntries.size()))
            return;
        const LmapEntry &entry = cellEntries[rec.pi];
        const LumelGrid grid =
            buildLumelGrid(*mWr, rec.ci, rec.pi, mDensity);
        if (!grid.valid || grid.lx != rec.w || grid.ly != rec.h) return;

        // Door overlays on this rec (light index + buffer).
        struct DoorOv { const std::vector<uint8_t> *buf; Vector3 loc; };
        std::vector<DoorOv> doorOvs;
        for (size_t k = 0; k < rec.overlayLightIdx.size(); ++k) {
            const int16_t li = rec.overlayLightIdx[k];
            if (li > 0 && static_cast<size_t>(li) < mExtraLights.size() &&
                mExtraLights[li] && k < rec.overlays.size())
                doorOvs.push_back(
                    {&rec.overlays[k],
                     mWr->staticLights[static_cast<size_t>(li)].loc});
        }
        if (doorOvs.empty()) return;

        std::vector<DirAccum> polyDir(
            static_cast<size_t>(rec.w) * rec.h, DirAccum{});
        for (int j = 0; j < rec.h; ++j) {
            for (int i = 0; i < rec.w; ++i) {
                const size_t t = static_cast<size_t>(j) * rec.w + i;
                const uint8_t *d4 = &rec.dirBase[t * 4];
                DirAccum &da = polyDir[t];
                const float mag =
                    d4[2] / 255.0f * rec.dirBaseMagScale;
                if (mag > 0.0f)
                    da.sum = octahedralDecode(d4[0] / 255.0f,
                                              d4[1] / 255.0f) * mag;
                da.weight = d4[3] / 255.0f * rec.dirBaseWeightScale;
                const Vector3 wp = grid.at(i, j);
                for (const DoorOv &ov : doorOvs) {
                    const uint8_t *px = &(*ov.buf)[t * 3];
                    const float lum =
                        std::max({px[0], px[1], px[2]}) / 255.0f;
                    if (lum <= 0.0f) continue;
                    const Vector3 toL = ov.loc - wp;
                    const float len = glm::length(toL);
                    if (len < 1e-6f) continue;
                    da.sum += (toL / len) * lum;
                    da.weight += lum;
                }
            }
        }
        writeDirTexelsToAtlas(polyDir, entry, *mDirAtlas, mDensity,
                              /*preserveAlpha=*/true);
        mDirDirty.push_back(recIdx);
    }

    // ── S4c promotion machinery ─────────────────────────────────────────
    // One baked light rendered differentially while a door swings
    // through its cone. frozenId's faces hold the door at the
    // LAST-BAKED pose (pinned, rendered once per acquire); dynId's
    // faces track the current pose via the caster hash.
    struct PromotedLight {
        int16_t lightIdx = -1;
        int     frozenId = -1;
        int     dynId = -1;
        int     slotFrozen = -1;
        int     slotCurrent = -1;
        std::unordered_set<int32_t> doors;
    };

    bool doorMoving(int32_t objID,
                    std::chrono::steady_clock::time_point now) const {
        auto it = mLastMotion.find(objID);
        return it != mLastMotion.end() &&
               std::chrono::duration<float>(now - it->second).count() <
                   kSettleSec;
    }

    // The bake's per-light intensity factor — same math as the event
    // submit path (K folded into the colour, shader stays formula-free).
    float promoteK(int16_t lightIdx) const {
        const float anchor = mFormula.anchorFor(lightIdx);
        const float a2 = mFormula.emitterRadius * mFormula.emitterRadius;
        return anchor > 0.0f ? (anchor * anchor + a2) / anchor : 0.0f;
    }

    // Snapshot door body poses as the "baked pose" reference — all doors
    // at first sight (load pose == bake pose), the drained event's doors
    // after each settle re-bake. If a door re-moved mid-event the
    // snapshot reads the already-moving pose and the frozen reference is
    // briefly wrong; the next settle event corrects it (double-swing,
    // rare, transient).
    void snapshotBakedPoses(const std::unordered_set<int32_t> *only) {
        if (!mOcclusion.world) return;
        const auto &bodies = mOcclusion.world->getBodies();
        for (const DoorShadowDoor &d : mDoors) {
            if (only && !only->count(d.objID)) continue;
            if (d.bodyIdx >= bodies.size()) continue;
            const ObjectCollisionBody &b = bodies[d.bodyIdx];
            mBakedPoseSnap[d.objID] =
                ShadowCasterPoseOverride{d.bodyIdx, b.worldPos, b.rotation};
        }
    }

    std::vector<ShadowCasterPoseOverride> bakedPoseOverrides() const {
        std::vector<ShadowCasterPoseOverride> ov;
        ov.reserve(mBakedPoseSnap.size());
        for (const auto &kv : mBakedPoseSnap) ov.push_back(kv.second);
        return ov;
    }

    // Door began (or continues) moving: promote its cone lights. Also
    // fires the object-cache invalidation spheres during motion — with
    // events deferred to settle, motion-time invalidation no longer
    // rides the rebuild.
    void promoteDoorLights(ShadowMapCache &sc, int32_t objID,
                           uint32_t bgfxFrame,
                           std::chrono::steady_clock::time_point now,
                           std::vector<std::pair<Vector3, float>> &inval) {
        const DoorShadowDoor *d = nullptr;
        for (const auto &dd : mDoors)
            if (dd.objID == objID) { d = &dd; break; }
        if (!d) return;
        auto &lastInv = mLastInvalidate[objID];
        if (std::chrono::duration<float>(now - lastInv).count() >=
            kInvalidateSec) {
            lastInv = now;
            inval.push_back({d->pos, d->sweptRadius + kInvalidateMargin});
        }
        auto lw = mDoorWork.find(objID);
        if (lw == mDoorWork.end()) return;
        // Unique cone lights of this door, nearest to the leaf first —
        // when the cap truncates, the lights whose shadows the swing
        // changes most keep their slots.
        std::vector<int16_t> lights;
        for (const auto &pr : lw->second) {
            if (std::find(lights.begin(), lights.end(), pr.second) ==
                lights.end())
                lights.push_back(pr.second);
        }
        std::sort(lights.begin(), lights.end(),
                  [&](int16_t a, int16_t b) {
                      return glm::length(mWr->staticLights[a].loc - d->pos) <
                             glm::length(mWr->staticLights[b].loc - d->pos);
                  });
        for (int16_t li : lights) {
            PromotedLight *p = nullptr;
            for (auto &pp : mPromoted)
                if (pp.lightIdx == li) { p = &pp; break; }
            if (p) { p->doors.insert(objID); continue; }
            const WRStaticLight &L =
                mWr->staticLights[static_cast<size_t>(li)];
            // Spot lights: the live term is omni-only (same TODO as the
            // GPU bake shader) — they keep the settle-event path.
            if (L.inner != -1.0f) continue;
            const float reach =
                static_cast<size_t>(li) < mReach.size()
                    ? mReach[static_cast<size_t>(li)] : 0.0f;
            if (reach <= 0.0f) continue;
            if (mPromoted.size() >= kPromoteCap) {
                const auto nowLog = std::chrono::steady_clock::now();
                if (std::chrono::duration<float>(nowLog - mLastPromoteLog)
                        .count() >= 1.0f) {
                    mLastPromoteLog = nowLog;
                    std::fprintf(stderr,
                        "[DOOR_SHADOW] promotion cap %d full — light %d "
                        "(door %d) stays baked (lagged) this swing\n",
                        static_cast<int>(kPromoteCap),
                        static_cast<int>(li), objID);
                }
                continue;
            }
            PromotedLight np;
            np.lightIdx = li;
            np.frozenId = -3000 - static_cast<int>(li);
            np.dynId = -2000 - static_cast<int>(li);
            const auto ov = bakedPoseOverrides();
            np.slotFrozen = acquireFrozenShadowSlot(
                sc, *mWr, np.frozenId, L.loc, reach, ov, bgfxFrame);
            if (np.slotFrozen < 0) {
                std::fprintf(stderr,
                    "[DOOR_SHADOW] no S1 slot for frozen reference of "
                    "light %d — promotion skipped (pool exhausted?)\n",
                    static_cast<int>(li));
                continue;
            }
            np.doors.insert(objID);
            std::fprintf(stderr,
                "[DOOR_SHADOW] promote light %d for door %d "
                "(frozen slot %d)\n",
                static_cast<int>(li), objID, np.slotFrozen);
            mPromoted.push_back(std::move(np));
        }
    }

    // Settle event drained: refresh its doors' baked poses; drop those
    // doors from promotions (they are baked and still); refreeze lights
    // whose OTHER doors are still moving/pending against the new poses.
    void settleDrained(ShadowMapCache &sc, uint32_t bgfxFrame,
                       std::chrono::steady_clock::time_point now) {
        snapshotBakedPoses(&mEventDoors);
        for (auto it = mPromoted.begin(); it != mPromoted.end();) {
            PromotedLight &p = *it;
            bool refreeze = false;
            for (auto dit = p.doors.begin(); dit != p.doors.end();) {
                const int32_t dID = *dit;
                const bool baked = mEventDoors.count(dID) != 0;
                const bool pending = mDirtyDoors.count(dID) != 0 ||
                                     doorMoving(dID, now);
                if (baked && !pending) {
                    dit = p.doors.erase(dit);
                } else {
                    if (baked) refreeze = true;
                    ++dit;
                }
            }
            if (p.doors.empty()) {
                releaseShadowSlot(sc, p.frozenId);
                releaseShadowSlot(sc, p.dynId);
                std::fprintf(stderr, "[DOOR_SHADOW] demote light %d\n",
                             static_cast<int>(p.lightIdx));
                it = mPromoted.erase(it);
                continue;
            }
            if (refreeze) {
                const size_t li = static_cast<size_t>(p.lightIdx);
                const float reach =
                    li < mReach.size() ? mReach[li] : 0.0f;
                if (reach > 0.0f) {
                    const auto ov = bakedPoseOverrides();
                    p.slotFrozen = acquireFrozenShadowSlot(
                        sc, *mWr, p.frozenId,
                        mWr->staticLights[li].loc, reach, ov, bgfxFrame);
                }
            }
            ++it;
        }
        mEventDoors.clear();
    }

    static constexpr float kRebuildSec = 0.15f;
    // Motion is over when no pose change arrives for this long. Door
    // physics ticks at 12.5 Hz (80 ms gaps between markDirty calls
    // mid-swing) — 0.3 s cannot false-settle a moving leaf.
    static constexpr float kSettleSec = 0.30f;
    // Mirrors the shader's LIVE_LIGHT_CAP minus nothing — the frame
    // loop merges these with the flashlight and enforces the real cap,
    // logging any truncation.
    static constexpr size_t kPromoteCap = 4;
    static constexpr int kPolysPerFrame = 48;
    static constexpr double kFrameMsBudget = 4.0;
    // Object-cache invalidation: door-scale radius, at most once per
    // window per door. A distant object lit through the doorway keeps a
    // stale cache until it moves — accepted cut, recorded in the plan.
    static constexpr float kInvalidateMargin = 24.0f;
    static constexpr float kInvalidateSec = 0.75f;
    static constexpr size_t kGpuRectsPerBatch = 256;
    static constexpr double kDirMsBudget = 2.0;

    const WRParsedData *mWr = nullptr;
    const RenderParams *mRp = nullptr;
    BakeFormula mFormula;
    std::vector<float> mAnchors;
    std::vector<int32_t> mLightCells;
    std::vector<DoorShadowDoor> mDoors;
    std::vector<uint8_t> mExtraLights;
    DoorSegmentOcclusion mOcclusion;
    std::vector<RebakedAnimPoly> *mAnimPolys = nullptr;
    std::unordered_map<int16_t, std::vector<size_t>> mLightRecs;
    std::vector<glm::vec4> mRecSphere;   // xyz poly center, w radius
    AtlasTexture *mDirAtlas = nullptr;
    const LightmapAtlasSet *mAtlasSet = nullptr;
    int mDensity = 1;
    std::vector<float> mReach;
    std::vector<size_t> mDirDirty;
    std::vector<size_t> mDirQueue;
    bool mDirInitPainted = false;
public:
    bool dirInitPainted() const { return mDirInitPainted; }
private:
    std::unordered_set<size_t> mEventDirDone;
    size_t mEventGpuRects = 0;
    size_t mEventCpuRects = 0;

    std::unordered_set<int32_t> mDirtyDoors;
    std::unordered_set<int32_t> mProcessingDoors;
    std::unordered_map<int32_t, Matrix4> mLastPose;
    // S4c promotion state.
    std::vector<PromotedLight> mPromoted;
    std::vector<PromotedLive> mPromotedLive;
    std::unordered_map<int32_t,
                       std::chrono::steady_clock::time_point> mLastMotion;
    std::unordered_map<int32_t, ShadowCasterPoseOverride> mBakedPoseSnap;
    std::unordered_set<int32_t> mEventDoors;
    std::chrono::steady_clock::time_point mLastPromoteLog{};
    bool mPoseSnapInit = false;
    std::unordered_map<int32_t, std::vector<std::pair<size_t, int16_t>>>
        mDoorWork;
    std::unordered_map<int32_t,
                       std::chrono::steady_clock::time_point>
        mLastInvalidate;
    std::vector<std::pair<size_t, int16_t>> mWork;
    size_t mWorkCursor = 0;
    std::vector<size_t> mReadyRecs;
    std::chrono::steady_clock::time_point mLastRebuild{};
    std::chrono::steady_clock::time_point mLastLog{};
    size_t mEventPolysTotal = 0;
    uint64_t mEventRays = 0;
    double mEventMs = 0.0;
};

} // namespace Darkness
