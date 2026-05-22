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

#include "ProbeManager.h"

#include "AudioLog.h"
#include "ProbeFile.h"
#include "SteamAudioPathing.h"
#include "logger.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sys/stat.h>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#endif

#include <phonon.h>

namespace Darkness {

// Engine ↔ IPL coord bridge. Engine: RH Z-up (+X right, +Y forward, +Z up).
// IPL: RH Y-up (+X right, +Y up, -Z ahead). Mapping: engine (x,y,z) →
// IPL (-y, z, -x), det=+1 (handedness preserved). Distance fields crossing
// the boundary scale by kFeetToMeters. Matches AudioService.cpp helpers.
namespace {

inline IPLVector3 engineToIplPos(const Vector3 &p) {
    return { -p.y * kFeetToMeters,
              p.z * kFeetToMeters,
             -p.x * kFeetToMeters };
}
inline Vector3 iplToEnginePos(const IPLVector3 &p) {
    return Vector3(-p.z * kMetersToFeet,
                   -p.x * kMetersToFeet,
                    p.y * kMetersToFeet);
}

struct PlaceResult {
    bool    ok;
    Vector3 pos;
    int     iters;  // 0=accepted as-is; N>0=nudged N times (or hit cap if !ok)
};

/// Place a probe through the filter, nudging up to maxIters times. 4
/// iterations is generous — typical recovery converges in 1-2. Adds a
/// 0.5 ft margin to each nudge so the next eval lands inside the valid
/// region (avoids float-jitter ping-pong at the boundary).
inline PlaceResult tryPlaceProbe(Vector3 pos, const ProbeFilterFn &filter,
                                 int maxIters = 4)
{
    if (!filter) return { true, pos, 0 };
    constexpr float kNudgeMarginFt = 0.5f;
    Vector3 cur = pos;
    for (int it = 0; it <= maxIters; ++it) {
        ProbeFilterDecision d = filter(cur);
        switch (d.result) {
            case ProbeFilterResult::Accept:
                return { true, cur, it };
            case ProbeFilterResult::Reject:
                return { false, cur, it };
            case ProbeFilterResult::Nudge:
                // Degenerate direction → reject rather than spin.
                if (glm::dot(d.nudgeDir, d.nudgeDir) < 1e-6f) {
                    return { false, cur, it };
                }
                cur += d.nudgeDir * (d.nudgeDistFt + kNudgeMarginFt);
                break;
        }
    }
    return { false, cur, maxIters };
}

} // namespace

// ── Ctor / dtor ───────────────────────────────────────────────────────────

ProbeManager::ProbeManager(ProbeManagerDeps deps)
    : mDeps(std::move(deps))
{
}

ProbeManager::~ProbeManager()
{
    // Pure release — AudioService releases the simulator first, so no
    // simulator de-registration is needed here.
    if (mIplProbeBatch) {
        iplProbeBatchRelease(&mIplProbeBatch);
        mIplProbeBatch = nullptr;
        mProbeCount = 0;
    }
}

// ── Bake ──────────────────────────────────────────────────────────────────

bool ProbeManager::bakeProbes(IPLScene scene,
                              const std::string &outputPath,
                              const ProbeBakeParams &params,
                              std::atomic<float> *progress)
{
    if (!mDeps.context || !scene) {
        LOG_ERROR("ProbeManager: cannot bake probes — no acoustic scene");
        return false;
    }

    // Negative override → manager defaults.
    float spacing = (params.spacingFtOverride > 0.0f) ? params.spacingFtOverride : mProbeSpacingFt;
    float height  = (params.heightFtOverride  > 0.0f) ? params.heightFtOverride  : mProbeHeightFt;

    AUDIO_LOG( "Baking probes: spacing=%.1f height=%.1f "
               "(min_wall_clearance=%.1fft, filter=%s)\n",
                 spacing, height, params.minWallClearanceFt,
                 params.probeFilter ? "enabled" : "disabled");

    // Step 1: Generate probes on a uniform floor grid
    IPLProbeArray probeArray = nullptr;
    IPLerror err = iplProbeArrayCreate(mDeps.context, &probeArray);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplProbeArrayCreate failed (%d)", err);
        return false;
    }

    // UNIFORMFLOOR transform maps the UNIT CUBE [0,1]³ into world space (NOT
    // [-0.5,0.5]³ — using `center` as translation shifts the volume by
    // extent/2 and silently misplaces probes). OBB must be in IPL Y-up
    // space — raycasts go along IPL -Y to find floors. Expanded by one
    // spacing margin so probes cover near-edge areas.
    const Vector3 marginFt(spacing);
    Vector3 origMin = params.sceneMin - marginFt;       // engine-space, feet
    Vector3 origMax = params.sceneMax + marginFt;
    IPLVector3 iplCornerA = engineToIplPos(origMin);    // one IPL-space corner
    IPLVector3 iplCornerB = engineToIplPos(origMax);    // the opposite one
    IPLVector3 iplMin{ std::min(iplCornerA.x, iplCornerB.x),
                       std::min(iplCornerA.y, iplCornerB.y),
                       std::min(iplCornerA.z, iplCornerB.z) };
    IPLVector3 iplMax{ std::max(iplCornerA.x, iplCornerB.x),
                       std::max(iplCornerA.y, iplCornerB.y),
                       std::max(iplCornerA.z, iplCornerB.z) };

    IPLMatrix4x4 transform{};
    transform.elements[0][0] = iplMax.x - iplMin.x;  // IPL X scale (m)
    transform.elements[1][1] = iplMax.y - iplMin.y;  // IPL Y scale (m) — engine Z extent
    transform.elements[2][2] = iplMax.z - iplMin.z;  // IPL Z scale (m) — engine Y extent
    transform.elements[3][0] = iplMin.x;             // IPL X translation
    transform.elements[3][1] = iplMin.y;             // IPL Y translation
    transform.elements[3][2] = iplMin.z;             // IPL Z translation
    transform.elements[3][3] = 1.0f;

    AUDIO_LOG( "Scene bounds engine-ft: (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n"
               "Probe OBB engine-ft:     (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n"
               "Probe OBB IPL-m:         (%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)\n",
                 params.sceneMin.x, params.sceneMin.y, params.sceneMin.z,
                 params.sceneMax.x, params.sceneMax.y, params.sceneMax.z,
                 origMin.x, origMin.y, origMin.z,
                 origMax.x, origMax.y, origMax.z,
                 iplMin.x, iplMin.y, iplMin.z,
                 iplMax.x, iplMax.y, iplMax.z);

    IPLProbeGenerationParams genParams{};
    genParams.type = IPL_PROBEGENERATIONTYPE_UNIFORMFLOOR;
    // spacing/height are passed in feet; IPL grid is in meters.
    genParams.spacing = spacing * kFeetToMeters;
    genParams.height  = height  * kFeetToMeters;
    genParams.transform = transform;

    iplProbeArrayGenerateProbes(probeArray, scene, &genParams);

    int numProbes = iplProbeArrayGetNumProbes(probeArray);
    AUDIO_LOG( "Generated %d probes (spacing=%.1f, height=%.1f)\n",
                 numProbes, spacing, height);

    if (numProbes == 0) {
        LOG_ERROR("ProbeManager: no probes generated — check scene geometry");
        iplProbeArrayRelease(&probeArray);
        return false;
    }

    // Step 2: Create probe batch and add the generated probes
    IPLProbeBatch probeBatch = nullptr;
    err = iplProbeBatchCreate(mDeps.context, &probeBatch);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplProbeBatchCreate failed (%d)", err);
        iplProbeArrayRelease(&probeArray);
        return false;
    }

    // Per-probe filtered add — drops in-wall / wall-adjacent / unreachable
    // candidates BEFORE baking, so the .probes file never contains bad
    // probes. Placement passes APPEND to mProbePositions; the IPL batch
    // is populated in a single final loop after a global dedup pass
    // filters cross-pass overlaps. Index in mProbePositions = batch index
    // (sidecars + energy audit address by that index).
    mProbePositions.clear();
    mProbePositions.reserve(static_cast<size_t>(numProbes));
    int rejectedFloor = 0;
    int nudgedFloor   = 0;
    for (int i = 0; i < numProbes; ++i) {
        IPLSphere s = iplProbeArrayGetProbe(probeArray, i);
        Vector3 enginePos = iplToEnginePos(s.center);
        PlaceResult pr = tryPlaceProbe(enginePos, params.probeFilter);
        if (!pr.ok) {
            ++rejectedFloor;
            continue;
        }
        if (pr.iters > 0) ++nudgedFloor;
        mProbePositions.push_back(pr.pos);
    }
    iplProbeArrayRelease(&probeArray);  // generator output no longer needed

    const int floorKept = static_cast<int>(mProbePositions.size());
    AUDIO_LOG("Floor probes: %d kept (%d nudged), %d rejected by filter "
              "(%d candidates)\n",
              floorKept, nudgedFloor, rejectedFloor, numProbes);

    if (floorKept == 0) {
        // Misconfigured filter (clearance too aggressive); fail loudly
        // rather than write an empty .probes file that breaks reverb.
        LOG_ERROR("ProbeManager: filter rejected every floor probe "
                  "(%d candidates) — refusing to bake an empty batch. "
                  "Check audio.probes.min_wall_clearance_ft.",
                  numProbes);
        iplProbeBatchRelease(&probeBatch);
        return false;
    }
    // numProbes now refers to the actual batch size (elevation/portal add to this).
    numProbes = floorKept;

    // ── Extra coverage: elevation tier + portal-centric rings ──
    //
    // Pathing greedy-routes through the nearest visible probes. With
    // floor-only probes:
    //   - Elevated emitters (wall torches, ceiling lamps) snap to the
    //     floor probe under them — pathing reports their position as
    //     ground-level, distorting eqCoeffs near vertical edges.
    //   - Portal openings can be densely surrounded by floor probes
    //     that sit on either side of the wall but none in the doorway
    //     itself — pathing wraps around through adjacent grid probes
    //     and stretches chains.
    //
    // Both tiers are extra `iplProbeBatchAddProbe` calls with the same
    // sphere radius as floor probes (one `spacing` worth, in meters).
    // Steam Audio uses these in the visibility graph alongside the
    // floor probes — no special treatment needed at bake or runtime.

    // Mirror count for sidecar / debug overlay alignment.
    int extraProbeCount = 0;

    if (!params.additionalElevations.empty()) {
        // Sparse elevation: bin floor probes on a coarser (x,y) grid and
        // emit one elevation probe per non-empty bin (centroid). With
        // sparsityMul=2 + 5ft spacing → 10ft bins → 1:4 elevation:floor
        // ratio. Enough density for vertical inter-level connectivity
        // via probe-to-probe visibility.
        const size_t floorProbeCount = mProbePositions.size();
        const float  binSizeFt       = spacing
                                     * std::max(1.0f, params.elevationSparsityMul);
        const float  invBinSizeFt    = 1.0f / binSizeFt;

        struct BinAcc { Vector3 sum{0.0f, 0.0f, 0.0f}; int count = 0; };
        std::map<std::pair<int, int>, BinAcc> bins;
        for (size_t i = 0; i < floorProbeCount; ++i) {
            const Vector3 &p = mProbePositions[i];
            int bx = static_cast<int>(std::floor(p.x * invBinSizeFt));
            int by = static_cast<int>(std::floor(p.y * invBinSizeFt));
            auto &b = bins[{bx, by}];
            b.sum.x += p.x;
            b.sum.y += p.y;
            b.sum.z += p.z;
            b.count++;
        }

        int rejectedElev = 0;
        int nudgedElev   = 0;
        int addedElev    = 0;
        for (float elevFt : params.additionalElevations) {
            if (elevFt <= 0.0f) continue;
            for (auto &kv : bins) {
                const BinAcc &b = kv.second;
                const float invCount = 1.0f / static_cast<float>(b.count);
                Vector3 centroid(b.sum.x * invCount,
                                 b.sum.y * invCount,
                                 b.sum.z * invCount);
                centroid.z += elevFt;
                PlaceResult pr = tryPlaceProbe(centroid, params.probeFilter);
                if (!pr.ok) { ++rejectedElev; continue; }
                if (pr.iters > 0) ++nudgedElev;
                mProbePositions.push_back(pr.pos);
                ++extraProbeCount;
                ++addedElev;
            }
        }
        AUDIO_LOG("Added %d elevation-tier probes "
                  "(floor count=%zu, tiers=%zu, bins=%zu, sparsityMul=%.1f, "
                  "%d nudged, %d rejected by filter)\n",
                  addedElev, floorProbeCount,
                  params.additionalElevations.size(),
                  bins.size(), params.elevationSparsityMul,
                  nudgedElev, rejectedElev);
    }

    if (!params.portalAxes.empty()) {
        // Up to 2 axial-offset probes per portal. Drop candidates within
        // dedupRadiusFt of an existing floor/elevation probe (already
        // covered). Survivors are openings without nearby grid coverage
        // (windows, vents, isolated hatches).
        const float axialOffsetFt  = params.portalAxialOffsetFt;
        const float dedupRadiusFt  = params.portalDedupRadiusFt;
        const float dedupRadiusSq  = dedupRadiusFt * dedupRadiusFt;

        // Snapshot BEFORE adding portal probes — otherwise the second
        // candidate of a pair dedups against the first (the doorway
        // hotspot we're fixing).
        const size_t gridProbeCount = mProbePositions.size();

        int portalProbeCount = 0;
        int dedupedCount     = 0;
        int rejectedCount    = 0;
        int nudgedCount      = 0;
        for (const auto &axis : params.portalAxes) {
            const Vector3 candidates[2] = {
                axis.center + axis.normal * axialOffsetFt,
                axis.center - axis.normal * axialOffsetFt,
            };
            for (const Vector3 &p : candidates) {
                // O(grid) brute scan — at miss6 scale this is ~10M ops, fine.
                bool tooClose = false;
                for (size_t i = 0; i < gridProbeCount; ++i) {
                    Vector3 d = mProbePositions[i] - p;
                    if (glm::dot(d, d) < dedupRadiusSq) { tooClose = true; break; }
                }
                if (tooClose) { ++dedupedCount; continue; }

                // Portal anchors sit ~1ft off the doorway plane — wall-
                // adjacent by design. Nudge loop pushes anchors near
                // thick frames further into the room rather than dropping.
                PlaceResult pr = tryPlaceProbe(p, params.probeFilter);
                if (!pr.ok) { ++rejectedCount; continue; }
                if (pr.iters > 0) ++nudgedCount;

                mProbePositions.push_back(pr.pos);
                ++extraProbeCount;
                ++portalProbeCount;
            }
        }
        AUDIO_LOG("Added %d portal-axis probes around %zu portals "
                  "(axialOffset=%.1fft, dedupRadius=%.1fft, "
                  "%d deduped, %d nudged, %d rejected by filter)\n",
                  portalProbeCount, params.portalAxes.size(),
                  axialOffsetFt, dedupRadiusFt, dedupedCount,
                  nudgedCount, rejectedCount);
    }

    // ── Emitter-anchored pass ─────────────────────────────────────────
    //
    // Phase 4 made Steam Audio the sole authority for player audio
    // routing. Voices outside the floor/elevation/portal grid's
    // visibility radius (typical case: wall-mounted ambients whose
    // P$Position resolves inside the visual wall mesh) get the 0.1f
    // pathing sentinel forever and fall into the synthetic-bypass
    // branch in AudioService — i.e. they bypass IPLPathEffect and
    // play through IPLDirectEffect alone, which leaks through walls.
    //
    // Anchoring a probe at every persistent emitter closes that gap:
    // the pathing solver associates the source with a real graph node
    // and produces routed eqCoeffs. Subject to the global-dedup pass
    // below so a grid probe that already covers the emitter wins.
    if (!params.emitterPositions.empty()) {
        const float dedupRadiusSq = params.portalDedupRadiusFt
                                  * params.portalDedupRadiusFt;
        const size_t preEmitterCount = mProbePositions.size();
        const ProbeFilterFn &emitterFilter = params.emitterProbeFilter
                                           ? params.emitterProbeFilter
                                           : params.probeFilter;
        int emitterAdded    = 0;
        int emitterDeduped  = 0;
        int emitterNudged   = 0;
        int emitterRejected = 0;
        // Per-emitter placement history. Logged after the pass so
        // we can correlate each accepted emitter probe with its
        // pre-placement candidate (did it land where the source is?
        // how many nudges? what's its spatial neighborhood?).
        struct EmitterRecord {
            Vector3 candidate;
            Vector3 placed;
            int     iters;
            size_t  probeIdx;  // index into mProbePositions
        };
        std::vector<EmitterRecord> placed;
        placed.reserve(params.emitterPositions.size());
        for (const Vector3 &p : params.emitterPositions) {
            // Pass-local dedup against earlier-pass probes (floor /
            // elevation / portal). Global dedup runs again below and
            // covers cross-emitter overlap.
            bool tooClose = false;
            for (size_t i = 0; i < preEmitterCount; ++i) {
                Vector3 d = mProbePositions[i] - p;
                if (glm::dot(d, d) < dedupRadiusSq) { tooClose = true; break; }
            }
            if (tooClose) { ++emitterDeduped; continue; }
            PlaceResult pr = tryPlaceProbe(p, emitterFilter);
            if (!pr.ok) { ++emitterRejected; continue; }
            if (pr.iters > 0) ++emitterNudged;
            mProbePositions.push_back(pr.pos);
            placed.push_back({ p, pr.pos, pr.iters, mProbePositions.size() - 1 });
            ++extraProbeCount;
            ++emitterAdded;
        }
        AUDIO_LOG("Added %d emitter-anchored probes "
                  "(%zu candidates, %d deduped, %d nudged, %d rejected)\n",
                  emitterAdded,
                  params.emitterPositions.size(),
                  emitterDeduped, emitterNudged, emitterRejected);

        // Per-emitter diagnostic: report the placement history + the
        // spatial neighborhood (number of OTHER probes within useful
        // distances). Helps distinguish "spatially isolated in open
        // space" (no neighbors within visibility range → solver can't
        // form edges no matter the geometry) from "spatially packed
        // but visibility-test failing" (many neighbors, but rays don't
        // cross intervening geometry). Pre-global-dedup snapshot —
        // the dedup below may further winnow the picture, but at this
        // point we have every emitter we placed in the position
        // record. Threshold counts: 5 ft = within Steam Audio's
        // bakeParams.radius (probe sample sphere) — these probes
        // overlap our emitter's sphere directly. 15 ft = within ~3
        // spacings — visibility should be cheap. 30 ft = within ~6
        // spacings — visibility possible but increasingly statistical.
        const size_t totalSoFar = mProbePositions.size();
        constexpr float kNear5Sq  =  5.0f *  5.0f;
        constexpr float kNear15Sq = 15.0f * 15.0f;
        constexpr float kNear30Sq = 30.0f * 30.0f;
        for (const EmitterRecord &r : placed) {
            int n5 = 0, n15 = 0, n30 = 0;
            for (size_t i = 0; i < totalSoFar; ++i) {
                if (i == r.probeIdx) continue;
                Vector3 d = mProbePositions[i] - r.placed;
                float sq = glm::dot(d, d);
                if (sq < kNear5Sq)  ++n5;
                if (sq < kNear15Sq) ++n15;
                if (sq < kNear30Sq) ++n30;
            }
            float displacement = glm::length(r.placed - r.candidate);
            AUDIO_LOG("[EMITTER_PROBE] idx=%zu cand=(%.1f,%.1f,%.1f) "
                      "placed=(%.1f,%.1f,%.1f) displaced=%.1fft iters=%d "
                      "neighbors n5=%d n15=%d n30=%d\n",
                      r.probeIdx,
                      r.candidate.x, r.candidate.y, r.candidate.z,
                      r.placed.x, r.placed.y, r.placed.z,
                      displacement, r.iters,
                      n5, n15, n30);
        }
    }

    if (extraProbeCount > 0) {
        AUDIO_LOG("Probes after placement (pre-dedup): %zu "
                  "(floor=%d, extra=%d)\n",
                  mProbePositions.size(),
                  static_cast<int>(mProbePositions.size()) - extraProbeCount,
                  extraProbeCount);
    }

    // Global dedup: walk in placement order; drop probes within
    // globalDedupRadiusFt of an earlier-kept probe. Earlier passes
    // (floor → elevation → portal → emitter) win. Safety net for
    // cross-pass overlap. O(N²) at bake time only.
    if (params.globalDedupRadiusFt > 0.0f && mProbePositions.size() > 1) {
        const float dedupRadiusSq = params.globalDedupRadiusFt
                                  * params.globalDedupRadiusFt;
        std::vector<Vector3> kept;
        kept.reserve(mProbePositions.size());
        int globalDeduped = 0;
        for (const Vector3 &p : mProbePositions) {
            bool tooClose = false;
            for (const Vector3 &k : kept) {
                Vector3 d = k - p;
                if (glm::dot(d, d) < dedupRadiusSq) { tooClose = true; break; }
            }
            if (tooClose) { ++globalDeduped; continue; }
            kept.push_back(p);
        }
        if (globalDeduped > 0) {
            AUDIO_LOG("Global dedup: kept %zu / %zu probes "
                      "(%d deduped at %.1f ft)\n",
                      kept.size(), mProbePositions.size(),
                      globalDeduped, params.globalDedupRadiusFt);
        }
        mProbePositions = std::move(kept);
    }

    // Commit survivors to the IPL batch. Uniform radius (one floor
    // spacing in meters) matches bake `radius` and runtime `visRadius`
    // for consistent visibility across passes.
    {
        const float probeRadiusM = spacing * kFeetToMeters;
        for (const Vector3 &p : mProbePositions) {
            IPLSphere s{};
            s.center = engineToIplPos(p);
            s.radius = probeRadiusM;
            iplProbeBatchAddProbe(probeBatch, s);
        }
        numProbes = static_cast<int>(mProbePositions.size());
        AUDIO_LOG("Emitted %d probes to IPL batch\n", numProbes);
    }

    iplProbeBatchCommit(probeBatch);

    // Step 3: Bake pathing data (visibility graph + shortest paths)
    AUDIO_LOG( "Baking pathing data for %d probes...\n", numProbes);

    IPLBakedDataIdentifier pathId{};
    pathId.type = IPL_BAKEDDATATYPE_PATHING;
    pathId.variation = IPL_BAKEDDATAVARIATION_DYNAMIC;

    // Pathing bake — all distances are feet → meters at the IPL boundary.
    // radius/threshold/visRange MUST agree with runtime IPLSimulationInputs.
    IPLPathBakeParams bakeParams{};
    bakeParams.scene = scene;
    bakeParams.probeBatch = probeBatch;
    bakeParams.identifier = pathId;
    bakeParams.numSamples = 4;                                          // rays = numSamples²
    bakeParams.radius = pathingVisRadiusMeters(spacing);
    bakeParams.threshold = kPathingVisThreshold;
    bakeParams.visRange  = params.propagationMaxDist * kFeetToMeters;
    bakeParams.pathRange = params.propagationMaxDist * kFeetToMeters;
    bakeParams.numThreads = 4;

    auto bakeStart = std::chrono::steady_clock::now();

    iplPathBakerBake(mDeps.context, &bakeParams,
        [](IPLfloat32 p, void *userData) {
            auto *prog = static_cast<std::atomic<float> *>(userData);
            if (prog) prog->store(p, std::memory_order_relaxed);
        },
        progress);

    auto bakeEnd = std::chrono::steady_clock::now();
    float bakeSec = std::chrono::duration<float>(bakeEnd - bakeStart).count();
    AUDIO_LOG( "Pathing bake complete: %d probes in %.1f seconds\n",
                 numProbes, bakeSec);

    // Step 3b: Bake reflection IRs. REVERB variation = one bake covers
    // all sources (listener-position-based); lets voices outside the
    // realtime top-N use baked reverb instead of going dry.
    AUDIO_LOG( "Baking reflection IRs for %d probes (rays=%d bounces=%d duration=%.1fs diffuse=%d order=%d)...\n",
                 numProbes, params.bakeNumRays, params.bakeNumBounces,
                 params.bakeDuration, params.bakeDiffuseSamples,
                 params.ambisonicsOrder);

    IPLBakedDataIdentifier reflId{};
    reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
    reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;

    unsigned int hwThreads = std::thread::hardware_concurrency();
    int bakeThreads = (params.simulatorThreads > 0)
        ? params.simulatorThreads
        : std::max(2u, hwThreads > 2 ? hwThreads - 2 : 2u);

    IPLReflectionsBakeParams reflBakeParams{};
    reflBakeParams.scene = scene;
    reflBakeParams.probeBatch = probeBatch;
    reflBakeParams.sceneType = (params.sceneType == "embree")
        ? IPL_SCENETYPE_EMBREE : IPL_SCENETYPE_DEFAULT;
    reflBakeParams.identifier = reflId;
    // HYBRID needs BOTH flags: BAKECONVOLUTION = per-probe ambisonic IR
    // (early reflections); BAKEPARAMETRIC = per-probe 3-band RT60 (late
    // tail). Without PARAMETRIC, reverbTimes stays 0 → undefined decay
    // → "second reverb fully audible" artefact.
    reflBakeParams.bakeFlags = static_cast<IPLReflectionsBakeFlags>(
        IPL_REFLECTIONSBAKEFLAGS_BAKECONVOLUTION
        | IPL_REFLECTIONSBAKEFLAGS_BAKEPARAMETRIC);
    reflBakeParams.numRays = params.bakeNumRays;
    reflBakeParams.numDiffuseSamples = params.bakeDiffuseSamples;
    reflBakeParams.numBounces = params.bakeNumBounces;
    reflBakeParams.simulatedDuration = params.bakeDuration;
    reflBakeParams.savedDuration = params.bakeDuration;
    reflBakeParams.order = params.ambisonicsOrder;
    reflBakeParams.numThreads = bakeThreads;
    // Meters. Shared with runtime sim (AudioUnits.h) — bake and runtime
    // MUST agree on this singularity clamp or IR energies desync.
    reflBakeParams.irradianceMinDistance = kIrradianceMinDistanceMeters;

    auto reflBakeStart = std::chrono::steady_clock::now();

    iplReflectionsBakerBake(mDeps.context, &reflBakeParams,
        [](IPLfloat32 p, void *userData) {
            auto *prog = static_cast<std::atomic<float> *>(userData);
            if (prog) prog->store(p, std::memory_order_relaxed);
        },
        progress);

    auto reflBakeEnd = std::chrono::steady_clock::now();
    float reflBakeSec = std::chrono::duration<float>(reflBakeEnd - reflBakeStart).count();
    AUDIO_LOG( "Reflection bake complete: %d probes in %.1f seconds\n",
                 numProbes, reflBakeSec);

    // Step 3c: Per-probe IR energy audit. Baked IRs vary wildly in total
    // energy; "hot" probes (small hard-surfaced rooms) saturate the wet bus
    // when listened to. Dumps a `.energy.csv` sidecar + stderr min/median/
    // max summary so outliers are visible. Metrics per probe: totalEnergy,
    // peakBin (early-cluster signature), earlyEnergy (first 50ms),
    // bandEnergy[3] (per-frequency-band totals).
    {
        IPLEnergyFieldSettings efSettings{};
        efSettings.duration = params.bakeDuration;
        efSettings.order    = params.ambisonicsOrder;
        IPLEnergyField energyField = nullptr;
        IPLerror efErr = iplEnergyFieldCreate(mDeps.context, &efSettings,
                                              &energyField);
        if (efErr != IPL_STATUS_SUCCESS || !energyField) {
            LOG_INFO("ProbeManager: skipping IR energy audit — "
                     "iplEnergyFieldCreate returned %d", efErr);
        } else {
            int numChannels = iplEnergyFieldGetNumChannels(energyField);
            int numBins     = iplEnergyFieldGetNumBins(energyField);
            int numBands    = IPL_NUM_BANDS;

            std::string energyPath = outputPath + ".energy.csv";
            FILE *ef = std::fopen(energyPath.c_str(), "w");
            if (ef) {
                std::fprintf(ef, "# Per-probe IR energy metrics — written by "
                                 "ProbeManager::bakeProbes.\n"
                                 "# duration=%.2fs order=%d channels=%d "
                                 "bands=%d bins=%d (10ms per bin)\n",
                             params.bakeDuration, params.ambisonicsOrder,
                             numChannels, numBands, numBins);
                std::fprintf(ef, "index,x,y,z,totalEnergy,peakBin,"
                                 "earlyEnergy50ms,bandLow,bandMid,bandHigh\n");
            }

            std::vector<float> totals(numProbes, 0.0f);
            int earlyBins = std::min(5, numBins);  // first 50 ms

            for (int p = 0; p < numProbes; ++p) {
                iplEnergyFieldReset(energyField);
                iplProbeBatchGetEnergyField(probeBatch, &reflId, p,
                                             energyField);

                // Only W channel (channel 0) — omni energy, non-negative.
                // Higher-order channels are signed SH coefficients; summing
                // them produces meaningless near-zero totals as harmonics
                // cancel. W is proportional to the angular integral of
                // incident energy = the per-probe "total reverb" metric.
                double totalEnergy = 0.0;
                double earlyEnergy = 0.0;
                float  peakBin     = 0.0f;
                double bandEnergy[IPL_NUM_BANDS] = {0.0, 0.0, 0.0};

                for (int b = 0; b < numBands; ++b) {
                    const float *bins = iplEnergyFieldGetBand(
                        energyField, /*channel=*/0, b);
                    if (!bins) continue;
                    for (int t = 0; t < numBins; ++t) {
                        float v = bins[t];
                        totalEnergy   += v;
                        bandEnergy[b] += v;
                        if (t < earlyBins) earlyEnergy += v;
                        if (v > peakBin) peakBin = v;
                    }
                }
                totals[p] = static_cast<float>(totalEnergy);

                if (ef && p < static_cast<int>(mProbePositions.size())) {
                    const auto &pos = mProbePositions[p];
                    std::fprintf(ef,
                        "%d,%.3f,%.3f,%.3f,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e\n",
                        p, pos.x, pos.y, pos.z,
                        totalEnergy, peakBin, earlyEnergy,
                        bandEnergy[0], bandEnergy[1], bandEnergy[2]);
                }
            }

            if (ef) {
                std::fclose(ef);
                AUDIO_LOG("Wrote probe IR energy sidecar '%s' (%d rows)\n",
                          energyPath.c_str(), numProbes);
            }

            // Stderr summary — outliers >10× median are the suspect probes
            // for runaway reverb when the listener parks near them.
            if (numProbes > 0) {
                std::vector<int> sorted(numProbes);
                for (int i = 0; i < numProbes; ++i) sorted[i] = i;
                std::sort(sorted.begin(), sorted.end(),
                          [&](int a, int b){ return totals[a] < totals[b]; });
                int minIdx = sorted.front();
                int maxIdx = sorted.back();
                int medIdx = sorted[numProbes / 2];
                float minE = totals[minIdx];
                float medE = totals[medIdx];
                float maxE = totals[maxIdx];
                int hotCount = 0;
                float hotThreshold = medE * 10.0f;
                for (float t : totals)
                    if (t > hotThreshold) ++hotCount;
                AUDIO_LOG("[PROBE_ENERGY] min=%.3e (probe %d) "
                          "median=%.3e (probe %d) max=%.3e (probe %d) "
                          "hot(>10×median)=%d/%d\n",
                          minE, minIdx, medE, medIdx, maxE, maxIdx,
                          hotCount, numProbes);
            }

            iplEnergyFieldRelease(&energyField);
        }
    }

    // Step 4: Serialize to memory buffer
    IPLSerializedObjectSettings soSettings{};
    soSettings.data = nullptr;
    soSettings.size = 0;

    IPLSerializedObject serializedObject = nullptr;
    err = iplSerializedObjectCreate(mDeps.context, &soSettings, &serializedObject);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplSerializedObjectCreate failed (%d)", err);
        iplProbeBatchRelease(&probeBatch);
        return false;
    }

    iplProbeBatchSave(probeBatch, serializedObject);

    IPLsize dataSize = iplSerializedObjectGetSize(serializedObject);
    IPLbyte *data = iplSerializedObjectGetData(serializedObject);

    // Round-trip validate before writing to disk.
    {
        IPLSerializedObjectSettings rtSettings{};
        rtSettings.data = data;
        rtSettings.size = dataSize;
        IPLSerializedObject rtObj = nullptr;
        IPLProbeBatch rtBatch = nullptr;
        err = iplSerializedObjectCreate(mDeps.context, &rtSettings, &rtObj);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ProbeManager: round-trip validation failed — "
                      "iplSerializedObjectCreate returned %d", err);
            iplSerializedObjectRelease(&serializedObject);
            iplProbeBatchRelease(&probeBatch);
            return false;
        }
        err = iplProbeBatchLoad(mDeps.context, rtObj, &rtBatch);
        iplSerializedObjectRelease(&rtObj);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ProbeManager: round-trip validation failed — "
                      "iplProbeBatchLoad returned %d", err);
            iplSerializedObjectRelease(&serializedObject);
            iplProbeBatchRelease(&probeBatch);
            return false;
        }
        int rtCount = iplProbeBatchGetNumProbes(rtBatch);
        iplProbeBatchRelease(&rtBatch);
        if (rtCount != numProbes) {
            LOG_ERROR("ProbeManager: round-trip validation failed — "
                      "expected %d probes, deserialized %d", numProbes, rtCount);
            iplSerializedObjectRelease(&serializedObject);
            iplProbeBatchRelease(&probeBatch);
            return false;
        }
        AUDIO_LOG( "Round-trip validation passed (%d probes)\n", rtCount);
    }

    // Step 6: Write to disk with integrity header (atomic tmp+rename)
    bool writeOk = writeProbeFile(
        outputPath,
        reinterpret_cast<const uint8_t *>(data),
        static_cast<size_t>(dataSize),
        static_cast<uint32_t>(numProbes));

    iplSerializedObjectRelease(&serializedObject);
    iplProbeBatchRelease(&probeBatch);

    if (!writeOk) {
        LOG_ERROR("ProbeManager: failed to write probe file '%s'", outputPath.c_str());
        return false;
    }

    AUDIO_LOG( "Saved %d probes to '%s' (%zu bytes + %zu header)\n",
                 numProbes, outputPath.c_str(), static_cast<size_t>(dataSize),
                 kProbeFileHeaderSize);

    // CSV sidecar — Steam Audio doesn't expose per-probe positions on a
    // deserialized batch, so we mirror them. Failure is non-fatal (the
    // overlay just lacks positions until the next bake).
    {
        std::string posPath = outputPath + ".positions.csv";
        FILE *pf = std::fopen(posPath.c_str(), "w");
        if (pf) {
            std::fprintf(pf, "# Probe positions in engine feet — written by "
                              "ProbeManager::bakeProbes. spacing=%.2f height=%.2f\n",
                          spacing, height);
            std::fprintf(pf, "index,x,y,z\n");
            for (size_t i = 0; i < mProbePositions.size(); ++i) {
                const auto &p = mProbePositions[i];
                std::fprintf(pf, "%zu,%.3f,%.3f,%.3f\n", i, p.x, p.y, p.z);
            }
            std::fclose(pf);
            AUDIO_LOG("Wrote probe positions sidecar '%s' (%zu rows)\n",
                      posPath.c_str(), mProbePositions.size());
        } else {
            LOG_INFO("ProbeManager: could not write probe positions sidecar '%s' "
                     "(debug overlay will be empty until next bake)",
                     posPath.c_str());
        }
    }
    return true;
}

// ── Load ──────────────────────────────────────────────────────────────────

bool ProbeManager::loadProbes(const std::string &probePath,
                              IPLSimulator reflectionSimulator)
{
    if (!mDeps.context || !reflectionSimulator) {
        LOG_ERROR("ProbeManager: cannot load probes — no context/simulator");
        return false;
    }

    // Release any previously loaded probes
    if (mIplProbeBatch) {
        if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();
        iplSimulatorRemoveProbeBatch(reflectionSimulator, mIplProbeBatch);
        iplSimulatorCommit(reflectionSimulator);
        iplProbeBatchRelease(&mIplProbeBatch);
        mIplProbeBatch = nullptr;
        mProbeCount = 0;
    }
    // Drop stale positions; we'll repopulate from the sidecar (or leave empty
    // if missing — overlay simply won't show anything in that case).
    mProbePositions.clear();
    mProbesHaveReflections = false;

    // Load and validate the probe file (header + CRC check)
    ProbeFileHeader hdr;
    std::vector<uint8_t> payload;
    ProbeFileStatus status = loadProbeFile(probePath, hdr, payload);

    if (status == ProbeFileStatus::FileNotFound) {
        LOG_INFO("ProbeManager: no probe file at '%s' — pathing disabled",
                 probePath.c_str());
        return false;
    }

    if (status != ProbeFileStatus::Ok) {
        LOG_ERROR("ProbeManager: probe file '%s' failed validation: %s — "
                  "delete and re-bake",
                  probePath.c_str(), probeFileStatusString(status));
        return false;
    }

    // Deserialize the validated payload
    IPLSerializedObjectSettings soSettings{};
    soSettings.data = reinterpret_cast<IPLbyte *>(payload.data());
    soSettings.size = static_cast<IPLsize>(payload.size());

    IPLSerializedObject serializedObject = nullptr;
    IPLerror err = iplSerializedObjectCreate(mDeps.context, &soSettings, &serializedObject);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplSerializedObjectCreate failed (%d) loading '%s'",
                  err, probePath.c_str());
        return false;
    }

    err = iplProbeBatchLoad(mDeps.context, serializedObject, &mIplProbeBatch);
    iplSerializedObjectRelease(&serializedObject);

    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplProbeBatchLoad failed (%d) for '%s'",
                  err, probePath.c_str());
        return false;
    }

    iplProbeBatchCommit(mIplProbeBatch);
    mProbeCount = iplProbeBatchGetNumProbes(mIplProbeBatch);

    // Cross-check: probe count from header must match what Steam Audio loaded
    if (mProbeCount != static_cast<int>(hdr.probeCount)) {
        LOG_ERROR("ProbeManager: probe count mismatch — header says %u, "
                  "Steam Audio loaded %d — discarding",
                  hdr.probeCount, mProbeCount);
        iplProbeBatchRelease(&mIplProbeBatch);
        mIplProbeBatch = nullptr;
        mProbeCount = 0;
        return false;
    }

    // Register with simulator
    if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();
    iplSimulatorAddProbeBatch(reflectionSimulator, mIplProbeBatch);
    iplSimulatorCommit(reflectionSimulator);

    // Reflection IRs present? Non-top-N voices use baked reverb when yes.
    IPLBakedDataIdentifier reflId{};
    reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
    reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;
    IPLsize reflDataSize = iplProbeBatchGetDataSize(mIplProbeBatch, &reflId);
    mProbesHaveReflections = (reflDataSize > 0);

    // Pathing graph present? A batch can have one without the other.
    // Missing graph → iplSimulatorRunPathing returns eqCoeffs=[0,0,0]
    // (audible as per-callback choppy gaps in the wet bus on ambients).
    IPLBakedDataIdentifier pathId{};
    pathId.type = IPL_BAKEDDATATYPE_PATHING;
    pathId.variation = IPL_BAKEDDATAVARIATION_DYNAMIC;
    IPLsize pathDataSize = iplProbeBatchGetDataSize(mIplProbeBatch, &pathId);

    AUDIO_LOG( "ProbeManager: loaded %d probes from '%s' "
                 "(reflections=%s, refl_size=%zu bytes, "
                 "pathing=%s, path_size=%zu bytes, crc=0x%08x)\n",
                 mProbeCount, probePath.c_str(),
                 mProbesHaveReflections ? "yes" : "no",
                 static_cast<size_t>(reflDataSize),
                 (pathDataSize > 0) ? "yes" : "no",
                 static_cast<size_t>(pathDataSize),
                 hdr.crc32);

    // Load positions sidecar (Steam Audio doesn't expose per-probe positions
    // on a deserialized batch). Missing/stale sidecar disables the overlay
    // only — the IPL batch itself is fine.
    std::string posPath = probePath + ".positions.csv";
    FILE *pf = std::fopen(posPath.c_str(), "r");
    if (pf) {
        char line[256];
        std::vector<Vector3> loaded;
        loaded.reserve(static_cast<size_t>(mProbeCount));
        while (std::fgets(line, sizeof(line), pf)) {
            if (line[0] == '#' || line[0] == 'i') continue;  // comment or header row
            int idx = 0;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (std::sscanf(line, "%d,%f,%f,%f", &idx, &x, &y, &z) == 4) {
                loaded.emplace_back(x, y, z);
            }
        }
        std::fclose(pf);
        if (static_cast<int>(loaded.size()) == mProbeCount) {
            mProbePositions = std::move(loaded);
            AUDIO_LOG("ProbeManager: loaded %zu probe positions from sidecar '%s'\n",
                      mProbePositions.size(), posPath.c_str());
        } else {
            LOG_INFO("ProbeManager: probe positions sidecar count mismatch "
                     "(%zu in sidecar vs %d in batch) — overlay disabled until re-bake",
                     loaded.size(), mProbeCount);
        }
    } else {
        LOG_INFO("ProbeManager: no probe positions sidecar at '%s' — "
                 "overlay disabled until re-bake (existing probe data is fine)",
                 posPath.c_str());
    }
    return true;
}

// ── Release ───────────────────────────────────────────────────────────────

void ProbeManager::releaseBatch(IPLSimulator reflectionSimulator)
{
    if (!mIplProbeBatch) return;

    if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();

    if (reflectionSimulator) {
        iplSimulatorRemoveProbeBatch(reflectionSimulator, mIplProbeBatch);
    }
    iplProbeBatchRelease(&mIplProbeBatch);
    mIplProbeBatch = nullptr;
    mProbeCount = 0;
    mProbesHaveReflections = false;
    mProbePositions.clear();
}

// ── Mission-relative probe file path ──────────────────────────────────────

std::string ProbeManager::getProbeFilePath(const std::string &misPath,
                                            const std::string &gameName)
{
    // Mission filename without extension, lowercased.
    std::string missionName;
    const auto lastSlash = misPath.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? misPath.substr(lastSlash + 1) : misPath;
    const auto dotPos = filename.rfind('.');
    missionName = (dotPos != std::string::npos) ? filename.substr(0, dotPos) : filename;
    for (auto &c : missionName) c = static_cast<char>(std::tolower(c));

    // ~/darkness/{gameName}/baked_probes/{missionName}.probes — mkdir -p.
    const char *home = std::getenv("HOME");
    if (!home) home = ".";
    const std::string dir = std::string(home) + "/darkness/" + gameName + "/baked_probes";

    std::string pathSoFar;
    for (size_t i = 0; i < dir.size(); ++i) {
        if (dir[i] == '/' || i == dir.size() - 1) {
            pathSoFar = dir.substr(0, i + 1);
#ifdef _WIN32
            _mkdir(pathSoFar.c_str());
#else
            mkdir(pathSoFar.c_str(), 0755);
#endif
        }
    }

    return dir + "/" + missionName + ".probes";
}

} // namespace Darkness
