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
        // Per-rec peaks (max byte of base / each overlay) — the DIRECTION
        // re-bake gate: re-encoding a poly's direction costs an
        // all-candidates pass (measured 12x the energy pass: 17.4M rays /
        // 9.1 s per one-door event ungated), and it only MATTERS where
        // this light meaningfully shapes the field.
        mRecBasePeak.clear();
        mRecOverlayPeak.clear();
        if (mAnimPolys) {
            mRecBasePeak.resize(mAnimPolys->size(), 0);
            mRecOverlayPeak.resize(mAnimPolys->size());
            for (size_t r = 0; r < mAnimPolys->size(); ++r) {
                const RebakedAnimPoly &rec = (*mAnimPolys)[r];
                uint8_t bp = 0;
                for (uint8_t b : rec.baseCrop) bp = std::max(bp, b);
                mRecBasePeak[r] = bp;
                auto &ops = mRecOverlayPeak[r];
                ops.resize(rec.overlays.size(), 0);
                for (size_t k = 0; k < rec.overlays.size(); ++k)
                    for (uint8_t b : rec.overlays[k])
                        ops[k] = std::max(ops[k], b);
            }
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
    std::vector<std::pair<Vector3, float>> update() {
        std::vector<std::pair<Vector3, float>> rebuiltFor;
        if (!active()) return rebuiltFor;
        const auto now = std::chrono::steady_clock::now();

        // Immediate rebuild when idle (kills the first-frame lag the user
        // reported); rate-limited only while an event is in flight.
        const bool idle = mWork.empty();
        if (!mDirtyDoors.empty() &&
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
            // Rebuild census — the line that distinguishes "no events
            // arrive" from "events arrive but select no work" (both look
            // like a dead feature without it). Cadence-limited by the
            // rebuild rate itself.
            std::fprintf(stderr,
                "[DOOR_SHADOW] rebuild: %zu (poly,light) work items "
                "(%zu door invalidation sphere(s))\n",
                mWork.size(), rebuiltFor.size());
        }

        // Burn the budget — bounded in BOTH polys and milliseconds (the
        // per-poly cost varies with lumel count and penumbra edges).
        int budget = kPolysPerFrame;
        const auto t0 = std::chrono::steady_clock::now();
        while (budget-- > 0 && mWorkCursor < mWork.size()) {
            const auto &[recIdx, lightIdx] = mWork[mWorkCursor++];
            rebakeOverlay(recIdx, lightIdx);
            if (dirSignificant(recIdx, lightIdx) &&
                mEventDirDone.insert(recIdx).second)
                rebakeDirection(recIdx);
            if (std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count() >
                kFrameMsBudget)
                break;
        }
        if (mWork.size() && mWorkCursor >= mWork.size()) {
            mEventMs += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            // Event complete — report once, rate-limited.
            const auto nowLog = std::chrono::steady_clock::now();
            if (std::chrono::duration<float>(nowLog - mLastLog).count() >=
                1.0f) {
                mLastLog = nowLog;
                std::fprintf(stderr,
                    "[DOOR_SHADOW] event: %zu overlay polys re-baked "
                    "(%llu rays, %.1f ms total across frames)\n",
                    mEventPolysTotal,
                    static_cast<unsigned long long>(mEventRays), mEventMs);
            }
            mWork.clear();
            mWorkCursor = 0;
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

    // Does this light shape the poly's field enough for its door state to
    // matter to the DIRECTION texels? Peaks precomputed at init.
    bool dirSignificant(size_t recIdx, int16_t lightIdx) const {
        if (recIdx >= mRecOverlayPeak.size() || !mAnimPolys) return false;
        const RebakedAnimPoly &rec = (*mAnimPolys)[recIdx];
        for (size_t k = 0; k < rec.overlayLightIdx.size(); ++k) {
            if (rec.overlayLightIdx[k] != lightIdx) continue;
            const uint8_t op = k < mRecOverlayPeak[recIdx].size()
                                   ? mRecOverlayPeak[recIdx][k] : 0;
            // Visible (>= ~6% full) AND at least half the base's peak.
            return op >= 16 && op * 2 >= mRecBasePeak[recIdx];
        }
        return false;
    }

    // Re-encode one poly's DIRECTION texels at current door poses: a
    // dir-only bake over ALL of the poly's candidate lights (the dir
    // texel is a weighted average across every contributor, so a
    // single-light pass cannot update it). Mirrors the load composition:
    // sun included, ambient excluded (directionless), door hook applies
    // to door-adjacent lights via the formula's extraOverlayLights gate.
    // Openness alpha is PRESERVED (the gather's, not re-derivable here).
    void rebakeDirection(size_t recIdx) {
        if (!mDirAtlas || !mAtlasSet || !mAnimPolys ||
            recIdx >= mAnimPolys->size())
            return;
        const RebakedAnimPoly &rec = (*mAnimPolys)[recIdx];
        if (rec.ci >= mAtlasSet->entries.size()) return;
        const auto &cellEntries = mAtlasSet->entries[rec.ci];
        if (rec.pi < 0 ||
            rec.pi >= static_cast<int>(cellEntries.size()))
            return;
        const LmapEntry &entry = cellEntries[rec.pi];

        // Candidate lights by reach vs the poly's bounding sphere; sun
        // joins if the cell's own authored list carries slot 0 (the same
        // rule the load bake applies).
        std::vector<uint16_t> cand;
        cand.push_back(0);
        const glm::vec4 &ps =
            recIdx < mRecSphere.size() ? mRecSphere[recIdx]
                                       : glm::vec4(0.0f);
        const Vector3 pc(ps.x, ps.y, ps.z);
        {
            const auto &own = mWr->cells[rec.ci].lightIndices;
            const int cnt = own.empty() ? 0 : static_cast<int>(own[0]);
            for (int k = 1; k <= cnt && k < static_cast<int>(own.size());
                 ++k)
                if (own[k] == 0) { cand.push_back(0); break; }
        }
        for (size_t li = 1; li < mReach.size(); ++li)
            if (mReach[li] > 0.0f &&
                glm::length(mWr->staticLights[li].loc - pc) <=
                    mReach[li] + ps.w)
                cand.push_back(static_cast<uint16_t>(li));
        cand[0] = static_cast<uint16_t>(cand.size() - 1);

        BakeFormula dirF = mFormula;
        dirF.includeSun = true;     // the load dir field includes it
        BakeStats stats;
        std::vector<Vector3> lumels;
        std::vector<DirAccum> polyDir(
            static_cast<size_t>(rec.w) * rec.h, DirAccum{});
        if (bakePolygon(*mWr, *mRp, rec.ci, rec.pi, dirF, mLightCells,
                        lumels, stats, nullptr, nullptr, &polyDir,
                        &cand) &&
            static_cast<int>(lumels.size()) == rec.w * rec.h) {
            writeDirTexelsToAtlas(polyDir, entry, *mDirAtlas, mDensity,
                                  /*preserveAlpha=*/true);
            mDirDirty.push_back(recIdx);
        }
        mEventRays += stats.rays;
    }

    static constexpr float kRebuildSec = 0.15f;
    static constexpr int kPolysPerFrame = 48;
    static constexpr double kFrameMsBudget = 4.0;
    // Object-cache invalidation: door-scale radius, at most once per
    // window per door. A distant object lit through the doorway keeps a
    // stale cache until it moves — accepted cut, recorded in the plan.
    static constexpr float kInvalidateMargin = 24.0f;
    static constexpr float kInvalidateSec = 0.75f;

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
    std::unordered_set<size_t> mEventDirDone;
    std::vector<uint8_t> mRecBasePeak;
    std::vector<std::vector<uint8_t>> mRecOverlayPeak;

    std::unordered_set<int32_t> mDirtyDoors;
    std::unordered_set<int32_t> mProcessingDoors;
    std::unordered_map<int32_t, Matrix4> mLastPose;
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
