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

/// Write a positions sidecar (used by the debug overlay since Steam Audio
/// doesn't expose per-probe positions on a deserialized batch). Failure is
/// non-fatal; we just lose the overlay until the next bake.
///
/// When `radii` is non-empty (and matches positions.size()) the optional
/// per-probe influence-radius column is appended. The reader gracefully
/// handles both 4- and 5-column rows so older sidecars stay loadable.
void writePositionsSidecar(const std::string &outputPath,
                           const std::string &suffix,
                           const std::vector<Vector3> &positions,
                           const std::vector<float>   &radii,
                           float spacing, float height)
{
    std::string posPath = outputPath + suffix + ".positions.csv";
    FILE *pf = std::fopen(posPath.c_str(), "w");
    if (!pf) {
        LOG_INFO("ProbeManager: could not write probe positions sidecar '%s' "
                 "(debug overlay will be empty until next bake)",
                 posPath.c_str());
        return;
    }
    const bool hasRadii = (radii.size() == positions.size());
    std::fprintf(pf, "# Probe positions in engine feet — written by "
                      "ProbeManager::bakeProbes. spacing=%.2f height=%.2f\n",
                  spacing, height);
    if (hasRadii) {
        std::fprintf(pf, "index,x,y,z,radiusFt\n");
    } else {
        std::fprintf(pf, "index,x,y,z\n");
    }
    for (size_t i = 0; i < positions.size(); ++i) {
        const auto &p = positions[i];
        if (hasRadii) {
            std::fprintf(pf, "%zu,%.3f,%.3f,%.3f,%.3f\n",
                         i, p.x, p.y, p.z, radii[i]);
        } else {
            std::fprintf(pf, "%zu,%.3f,%.3f,%.3f\n",
                         i, p.x, p.y, p.z);
        }
    }
    std::fclose(pf);
    AUDIO_LOG("Wrote probe positions sidecar '%s' (%zu rows, radii=%s)\n",
              posPath.c_str(), positions.size(),
              hasRadii ? "yes" : "no");
}

/// Read a positions sidecar written by writePositionsSidecar. Returns
/// empty on missing/malformed input — the IPL batch is fine without it,
/// only the overlay is affected.
///
/// Out param `radiiOut`: populated only when the sidecar carries the
/// optional 5th radius column. Older 4-column sidecars (and any row
/// that fails the 5-column scan) leave radiiOut empty so callers can
/// detect "radii unknown" via radiiOut.size() != positions.size().
std::vector<Vector3> readPositionsSidecar(const std::string &path,
                                          int expectedCount,
                                          std::vector<float> *radiiOut)
{
    std::vector<Vector3> loaded;
    if (radiiOut) radiiOut->clear();
    FILE *pf = std::fopen(path.c_str(), "r");
    if (!pf) return loaded;
    char line[256];
    loaded.reserve(static_cast<size_t>(expectedCount));
    if (radiiOut) radiiOut->reserve(static_cast<size_t>(expectedCount));
    bool radiiConsistent = true;
    while (std::fgets(line, sizeof(line), pf)) {
        if (line[0] == '#' || line[0] == 'i') continue;
        int idx = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f, r = 0.0f;
        int got = std::sscanf(line, "%d,%f,%f,%f,%f", &idx, &x, &y, &z, &r);
        if (got >= 4) {
            loaded.emplace_back(x, y, z);
            if (radiiOut) {
                if (got == 5) radiiOut->push_back(r);
                else          radiiConsistent = false;
            }
        }
    }
    std::fclose(pf);
    if (radiiOut && !radiiConsistent) radiiOut->clear();
    return loaded;
}

/// Serialize a probe batch and validate the round trip. Returns the
/// serialized payload bytes, or empty on failure.
std::vector<uint8_t> serializeBatchValidated(IPLContext context,
                                              IPLProbeBatch probeBatch,
                                              int expectedProbeCount,
                                              const char *label)
{
    std::vector<uint8_t> empty;

    IPLSerializedObjectSettings soSettings{};
    IPLSerializedObject serializedObject = nullptr;
    IPLerror err = iplSerializedObjectCreate(context, &soSettings, &serializedObject);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: [%s] iplSerializedObjectCreate failed (%d)",
                  label, err);
        return empty;
    }

    iplProbeBatchSave(probeBatch, serializedObject);

    IPLsize dataSize = iplSerializedObjectGetSize(serializedObject);
    IPLbyte *data = iplSerializedObjectGetData(serializedObject);

    // Round-trip validate
    {
        IPLSerializedObjectSettings rtSettings{};
        rtSettings.data = data;
        rtSettings.size = dataSize;
        IPLSerializedObject rtObj = nullptr;
        IPLProbeBatch rtBatch = nullptr;
        err = iplSerializedObjectCreate(context, &rtSettings, &rtObj);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ProbeManager: [%s] round-trip validation failed — "
                      "iplSerializedObjectCreate returned %d", label, err);
            iplSerializedObjectRelease(&serializedObject);
            return empty;
        }
        err = iplProbeBatchLoad(context, rtObj, &rtBatch);
        iplSerializedObjectRelease(&rtObj);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ProbeManager: [%s] round-trip validation failed — "
                      "iplProbeBatchLoad returned %d", label, err);
            iplSerializedObjectRelease(&serializedObject);
            return empty;
        }
        int rtCount = iplProbeBatchGetNumProbes(rtBatch);
        iplProbeBatchRelease(&rtBatch);
        if (rtCount != expectedProbeCount) {
            LOG_ERROR("ProbeManager: [%s] round-trip validation failed — "
                      "expected %d probes, deserialized %d",
                      label, expectedProbeCount, rtCount);
            iplSerializedObjectRelease(&serializedObject);
            return empty;
        }
        AUDIO_LOG("Round-trip validation passed [%s] (%d probes)\n",
                  label, rtCount);
    }

    std::vector<uint8_t> payload(static_cast<size_t>(dataSize));
    std::memcpy(payload.data(), data, dataSize);
    iplSerializedObjectRelease(&serializedObject);
    return payload;
}

} // namespace

// ── Ctor / dtor ───────────────────────────────────────────────────────────

ProbeManager::ProbeManager(ProbeManagerDeps deps)
    : mDeps(std::move(deps))
{
}

ProbeManager::~ProbeManager()
{
    // Pure release — AudioService releases the simulators first, so no
    // simulator de-registration is needed here.
    for (auto &entry : mBatches) {
        if (entry.iplBatch) {
            iplProbeBatchRelease(&entry.iplBatch);
            entry.iplBatch = nullptr;
        }
    }
    mBatches.clear();
    mReflectionsBatch = nullptr;
    mPathingBatch = nullptr;
    mTotalProbeCount = 0;
}

// ── Reflection batch bake ─────────────────────────────────────────────────

bool ProbeManager::bakeReflectionBatch(IPLScene scene,
                                       const ProbeBakeParams &params,
                                       float spacing,
                                       float height,
                                       std::atomic<float> *progress,
                                       const std::string &outputPath,
                                       ProbeBatchEntry &outEntry)
{
    AUDIO_LOG("Baking reflection batch: spacing=%.1f height=%.1f "
              "(min_wall_clearance=%.1fft, filter=%s)\n",
              spacing, height, params.minWallClearanceFt,
              params.probeFilter ? "enabled" : "disabled");

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
    Vector3 origMin = params.sceneMin - marginFt;
    Vector3 origMax = params.sceneMax + marginFt;
    IPLVector3 iplCornerA = engineToIplPos(origMin);
    IPLVector3 iplCornerB = engineToIplPos(origMax);
    IPLVector3 iplMin{ std::min(iplCornerA.x, iplCornerB.x),
                       std::min(iplCornerA.y, iplCornerB.y),
                       std::min(iplCornerA.z, iplCornerB.z) };
    IPLVector3 iplMax{ std::max(iplCornerA.x, iplCornerB.x),
                       std::max(iplCornerA.y, iplCornerB.y),
                       std::max(iplCornerA.z, iplCornerB.z) };

    IPLMatrix4x4 transform{};
    transform.elements[0][0] = iplMax.x - iplMin.x;
    transform.elements[1][1] = iplMax.y - iplMin.y;
    transform.elements[2][2] = iplMax.z - iplMin.z;
    transform.elements[3][0] = iplMin.x;
    transform.elements[3][1] = iplMin.y;
    transform.elements[3][2] = iplMin.z;
    transform.elements[3][3] = 1.0f;

    AUDIO_LOG("Reflection scene bounds engine-ft: (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n"
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
    genParams.spacing = spacing * kFeetToMeters;
    genParams.height  = height  * kFeetToMeters;
    genParams.transform = transform;

    iplProbeArrayGenerateProbes(probeArray, scene, &genParams);

    int numCandidates = iplProbeArrayGetNumProbes(probeArray);
    AUDIO_LOG("Reflection batch: generated %d floor candidates "
              "(spacing=%.1f, height=%.1f)\n",
              numCandidates, spacing, height);

    if (numCandidates == 0) {
        LOG_ERROR("ProbeManager: no probes generated — check scene geometry");
        iplProbeArrayRelease(&probeArray);
        return false;
    }

    IPLProbeBatch probeBatch = nullptr;
    err = iplProbeBatchCreate(mDeps.context, &probeBatch);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplProbeBatchCreate failed (%d)", err);
        iplProbeArrayRelease(&probeArray);
        return false;
    }

    std::vector<Vector3> &positions = outEntry.positions;
    positions.clear();
    positions.reserve(static_cast<size_t>(numCandidates));

    // Per-probe filtered add — drops in-wall / wall-adjacent / unreachable
    // candidates BEFORE baking, so the .probes file never contains bad
    // probes. Placement passes APPEND to positions; the IPL batch is
    // populated in a single final loop after a global dedup pass filters
    // cross-pass overlaps.
    int rejectedFloor = 0;
    int nudgedFloor   = 0;
    for (int i = 0; i < numCandidates; ++i) {
        IPLSphere s = iplProbeArrayGetProbe(probeArray, i);
        Vector3 enginePos = iplToEnginePos(s.center);
        PlaceResult pr = tryPlaceProbe(enginePos, params.probeFilter);
        if (!pr.ok) { ++rejectedFloor; continue; }
        if (pr.iters > 0) ++nudgedFloor;
        positions.push_back(pr.pos);
    }
    iplProbeArrayRelease(&probeArray);

    const int floorKept = static_cast<int>(positions.size());
    AUDIO_LOG("Floor probes: %d kept (%d nudged), %d rejected by filter "
              "(%d candidates)\n",
              floorKept, nudgedFloor, rejectedFloor, numCandidates);

    if (floorKept == 0) {
        // Misconfigured filter (clearance too aggressive); fail loudly
        // rather than write an empty .probes file that breaks reverb.
        LOG_ERROR("ProbeManager: filter rejected every floor probe "
                  "(%d candidates) — refusing to bake an empty batch. "
                  "Check audio.probes.min_wall_clearance_ft.",
                  numCandidates);
        iplProbeBatchRelease(&probeBatch);
        return false;
    }

    // ── Extra coverage: elevation tier + portal-centric rings ──
    int extraProbeCount = 0;

    if (!params.additionalElevations.empty()) {
        // Sparse elevation: bin floor probes on a coarser (x,y) grid and
        // emit one elevation probe per non-empty bin (centroid). With
        // sparsityMul=2 + 5ft spacing → 10ft bins → 1:4 elevation:floor
        // ratio. Enough density for vertical inter-level connectivity
        // via probe-to-probe visibility.
        const size_t floorProbeCount = positions.size();
        const float  binSizeFt       = spacing
                                     * std::max(1.0f, params.elevationSparsityMul);
        const float  invBinSizeFt    = 1.0f / binSizeFt;

        struct BinAcc { Vector3 sum{0.0f, 0.0f, 0.0f}; int count = 0; };
        std::map<std::pair<int, int>, BinAcc> bins;
        for (size_t i = 0; i < floorProbeCount; ++i) {
            const Vector3 &p = positions[i];
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
                positions.push_back(pr.pos);
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

    // (The per-portal probe-ring pass that lived here has been removed.
    // Pathing probes are now derived from RoomService on a separate
    // sparse batch — see ProbeManager::bakePathingBatch and
    // AudioService::bakeProbes' "Pathing-batch candidates" section.
    // params.portalAxialOffsetFt is still consumed there.)

    // ── Emitter-anchored pass ─────────────────────────────────────────
    if (!params.emitterPositions.empty()) {
        const float dedupRadiusSq = params.portalDedupRadiusFt
                                  * params.portalDedupRadiusFt;
        const size_t preEmitterCount = positions.size();
        const ProbeFilterFn &emitterFilter = params.emitterProbeFilter
                                           ? params.emitterProbeFilter
                                           : params.probeFilter;
        int emitterAdded    = 0;
        int emitterDeduped  = 0;
        int emitterNudged   = 0;
        int emitterRejected = 0;
        struct EmitterRecord {
            Vector3 candidate;
            Vector3 placed;
            int     iters;
            size_t  probeIdx;
        };
        std::vector<EmitterRecord> placed;
        placed.reserve(params.emitterPositions.size());
        for (const Vector3 &p : params.emitterPositions) {
            bool tooClose = false;
            for (size_t i = 0; i < preEmitterCount; ++i) {
                Vector3 d = positions[i] - p;
                if (glm::dot(d, d) < dedupRadiusSq) { tooClose = true; break; }
            }
            if (tooClose) { ++emitterDeduped; continue; }
            PlaceResult pr = tryPlaceProbe(p, emitterFilter);
            if (!pr.ok) { ++emitterRejected; continue; }
            if (pr.iters > 0) ++emitterNudged;
            positions.push_back(pr.pos);
            placed.push_back({ p, pr.pos, pr.iters, positions.size() - 1 });
            ++extraProbeCount;
            ++emitterAdded;
        }
        AUDIO_LOG("Added %d emitter-anchored probes "
                  "(%zu candidates, %d deduped, %d nudged, %d rejected)\n",
                  emitterAdded,
                  params.emitterPositions.size(),
                  emitterDeduped, emitterNudged, emitterRejected);

        // Per-emitter diagnostic
        const size_t totalSoFar = positions.size();
        constexpr float kNear5Sq  =  5.0f *  5.0f;
        constexpr float kNear15Sq = 15.0f * 15.0f;
        constexpr float kNear30Sq = 30.0f * 30.0f;
        for (const EmitterRecord &r : placed) {
            int n5 = 0, n15 = 0, n30 = 0;
            for (size_t i = 0; i < totalSoFar; ++i) {
                if (i == r.probeIdx) continue;
                Vector3 d = positions[i] - r.placed;
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
                  positions.size(),
                  static_cast<int>(positions.size()) - extraProbeCount,
                  extraProbeCount);
    }

    // Global dedup: walk in placement order; drop probes within
    // globalDedupRadiusFt of an earlier-kept probe.
    if (params.globalDedupRadiusFt > 0.0f && positions.size() > 1) {
        const float dedupRadiusSq = params.globalDedupRadiusFt
                                  * params.globalDedupRadiusFt;
        std::vector<Vector3> kept;
        kept.reserve(positions.size());
        int globalDeduped = 0;
        for (const Vector3 &p : positions) {
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
                      kept.size(), positions.size(),
                      globalDeduped, params.globalDedupRadiusFt);
        }
        positions = std::move(kept);
    }

    // Commit survivors to the IPL batch. Uniform radius (one floor
    // spacing in meters) matches bake `radius` and runtime `visRadius`
    // for consistent visibility across passes.
    {
        const float probeRadiusM = spacing * kFeetToMeters;
        for (const Vector3 &p : positions) {
            IPLSphere s{};
            s.center = engineToIplPos(p);
            s.radius = probeRadiusM;
            iplProbeBatchAddProbe(probeBatch, s);
        }
        AUDIO_LOG("Emitted %zu probes to reflection IPL batch\n",
                  positions.size());
    }

    iplProbeBatchCommit(probeBatch);
    const int numProbes = static_cast<int>(positions.size());

    // Reflection IRs. REVERB variation = one bake covers all sources
    // (listener-position-based); lets voices outside the realtime top-N
    // use baked reverb instead of going dry.
    AUDIO_LOG("Baking reflection IRs for %d probes "
              "(rays=%d bounces=%d duration=%.1fs diffuse=%d order=%d)...\n",
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
    AUDIO_LOG("Reflection bake complete: %d probes in %.1f seconds\n",
              numProbes, reflBakeSec);

    // Per-probe IR energy audit (unchanged from v1) — writes a `.energy.csv`
    // sidecar so outliers are visible in the post-bake summary.
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

            std::string energyPath = outputPath + ".reflections.energy.csv";
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
            int earlyBins = std::min(5, numBins);

            for (int p = 0; p < numProbes; ++p) {
                iplEnergyFieldReset(energyField);
                iplProbeBatchGetEnergyField(probeBatch, &reflId, p, energyField);

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

                if (ef && p < static_cast<int>(positions.size())) {
                    const auto &pos = positions[p];
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

    outEntry.purpose            = ProbePurpose::Reflections;
    outEntry.iplBatch           = probeBatch;
    outEntry.probeCount         = numProbes;
    outEntry.hasReflectionsData = true;
    outEntry.hasPathingData     = false;
    return true;
}

// ── Pathing batch bake ────────────────────────────────────────────────────

bool ProbeManager::bakePathingBatch(IPLScene scene,
                                    const ProbeBakeParams &params,
                                    float spacing,
                                    std::atomic<float> *progress,
                                    ProbeBatchEntry &outEntry)
{
    if (params.pathingCandidates.empty()) {
        AUDIO_LOG("Pathing batch: no candidates supplied — skipping bake\n");
        return false;
    }

    AUDIO_LOG("Baking pathing batch: %zu candidate nodes\n",
              params.pathingCandidates.size());

    IPLProbeBatch probeBatch = nullptr;
    IPLerror err = iplProbeBatchCreate(mDeps.context, &probeBatch);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: pathing batch iplProbeBatchCreate failed (%d)",
                  err);
        return false;
    }

    std::vector<Vector3> &positions = outEntry.positions;
    std::vector<float>   &radii     = outEntry.radiiFt;
    positions.clear();
    radii.clear();
    positions.reserve(params.pathingCandidates.size());
    radii.reserve(params.pathingCandidates.size());

    int rejected = 0;
    int nudged   = 0;
    for (const auto &cand : params.pathingCandidates) {
        PlaceResult pr = tryPlaceProbe(cand.position, params.pathingProbeFilter);
        if (!pr.ok) { ++rejected; continue; }
        if (pr.iters > 0) ++nudged;
        positions.push_back(pr.pos);
        radii.push_back(cand.radiusFt);
    }
    AUDIO_LOG("Pathing batch: kept %zu probes (%d nudged, %d rejected) "
              "from %zu candidates\n",
              positions.size(), nudged, rejected,
              params.pathingCandidates.size());

    if (positions.empty()) {
        LOG_ERROR("ProbeManager: filter rejected every pathing candidate — "
                  "refusing to bake an empty pathing batch");
        iplProbeBatchRelease(&probeBatch);
        return false;
    }

    // Commit candidates to the IPL batch. Per-candidate radius lets
    // room-centroid probes have a large influence sphere (covers the
    // whole room) while portal probes have a small one (so a closed-door
    // OBB between two portal probes invalidates only one edge instead of
    // the whole graph segment).
    for (size_t i = 0; i < positions.size(); ++i) {
        IPLSphere s{};
        s.center = engineToIplPos(positions[i]);
        s.radius = radii[i] * kFeetToMeters;
        iplProbeBatchAddProbe(probeBatch, s);
    }
    iplProbeBatchCommit(probeBatch);
    const int numProbes = static_cast<int>(positions.size());
    AUDIO_LOG("Emitted %d probes to pathing IPL batch\n", numProbes);

    // Bake pathing data (visibility graph + shortest paths). All
    // distances are feet → meters at the IPL boundary. radius / threshold
    // / visRange MUST agree with runtime IPLSimulationInputs (consumed by
    // AudioService).
    AUDIO_LOG("Baking pathing data for %d probes...\n", numProbes);

    IPLBakedDataIdentifier pathId{};
    pathId.type = IPL_BAKEDDATATYPE_PATHING;
    pathId.variation = IPL_BAKEDDATAVARIATION_DYNAMIC;

    IPLPathBakeParams bakeParams{};
    bakeParams.scene = scene;
    bakeParams.probeBatch = probeBatch;
    bakeParams.identifier = pathId;
    bakeParams.numSamples = 4;                                          // rays = numSamples²
    // kPathingVisRadiusFt is shared with the runtime call in
    // AudioService.cpp loopStep — they MUST agree. (void)spacing here
    // because the visibility-sphere radius is intentionally decoupled
    // from probe spacing — see the kPathingVisRadiusFt comment in
    // SteamAudioPathing.h.
    (void)spacing;
    bakeParams.radius = kPathingVisRadiusFt * kFeetToMeters;
    bakeParams.threshold = kPathingVisThreshold;
    // ── visRange / pathRange — bake-time graph budget ──────────────────
    //
    // visRange caps the maximum probe-to-probe distance at which the
    // bake will even consider building a visibility edge
    // (`ProbeVisibilityTester::areProbesTooFar`, path_visibility.cpp:95).
    // pathRange caps the total length of multi-hop shortest paths
    // stored in the BakedPathData. Both are PROBE-TO-PROBE distances
    // across the visibility GRAPH, NOT per-voice audible distances.
    //
    // Past sessions confused this with `mPropagationMaxDist` (the
    // per-voice audible cap consumed by reflections and AI hearing).
    // Tying the bake-time graph to a per-voice cap silently
    // truncated the graph: any two probes more than `propagationMaxDist`
    // apart had no edge in the baked graph at all, so a source and
    // listener separated by that distance got the 0.1f sentinel
    // forever even if a multi-hop path through nearer probes would
    // have reached them. The per-voice cap belongs at runtime
    // (`inputs.visRange` in AudioService.cpp) — at BAKE the graph
    // should span the whole level.
    //
    // We derive the bake-time range from the scene AABB diagonal so
    // the graph covers the entire level. Scaled by 1.5× to absorb
    // probes that landed slightly outside the floor-grid AABB
    // (portal probes flanking doors etc.) and clamped to a
    // generous-but-finite ceiling so a stray sceneMax doesn't
    // produce a meaningless multi-kilometer range. Matches Valve's
    // Unity convention (`bakingVisibilityRange = 1000 m` default) of
    // letting the solver walk the full graph; per-frame cost is
    // controlled via `numVisSamples` and the per-source runtime
    // `visRange`, NOT by truncating the bake graph.
    const Vector3 sceneSpan = params.sceneMax - params.sceneMin;
    const float sceneDiagonalFt = std::sqrt(sceneSpan.x * sceneSpan.x
                                          + sceneSpan.y * sceneSpan.y
                                          + sceneSpan.z * sceneSpan.z);
    constexpr float kBakeRangeMarginMul = 1.5f;
    constexpr float kBakeRangeCeilingFt = 5000.0f;  // ~1500 m — Valve-ish ceiling
    const float bakeRangeFt = std::min(
        std::max(sceneDiagonalFt * kBakeRangeMarginMul,
                 params.propagationMaxDist * kBakeRangeMarginMul),
        kBakeRangeCeilingFt);
    bakeParams.visRange  = bakeRangeFt * kFeetToMeters;
    bakeParams.pathRange = bakeRangeFt * kFeetToMeters;
    AUDIO_LOG("Pathing bake range: %.0f ft (sceneDiag=%.0f ft × %.1fx, "
              "ceiling=%.0f ft) → visRange=pathRange=%.1f m\n",
              bakeRangeFt, sceneDiagonalFt, kBakeRangeMarginMul,
              kBakeRangeCeilingFt, bakeRangeFt * kFeetToMeters);
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
    AUDIO_LOG("Pathing bake complete: %d probes in %.1f seconds\n",
              numProbes, bakeSec);

    outEntry.purpose            = ProbePurpose::Pathing;
    outEntry.iplBatch           = probeBatch;
    outEntry.probeCount         = numProbes;
    outEntry.hasReflectionsData = false;
    outEntry.hasPathingData     = true;
    return true;
}

// ── Top-level bake ────────────────────────────────────────────────────────

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

    // Bake the reflection batch.
    ProbeBatchEntry reflEntry;
    if (!bakeReflectionBatch(scene, params, spacing, height,
                             progress, outputPath, reflEntry)) {
        return false;
    }

    // Bake the pathing batch (optional). Failure to bake pathing is not
    // fatal — reflection batch alone still gives us reverb. Loud warning
    // so the operator sees pathing is disabled.
    ProbeBatchEntry pathEntry;
    bool havePathing = false;
    if (params.bakePathingBatch) {
        if (bakePathingBatch(scene, params, spacing, progress, pathEntry)) {
            havePathing = true;
        } else {
            AUDIO_LOG("[FALLBACK] pathing batch bake failed — file will "
                      "contain reflection batch only; Steam Audio pathing "
                      "disabled until re-bake\n");
        }
    }

    // Serialize each batch with round-trip validation, build the
    // ProbeBatchRecord list, write the v2 file.
    std::vector<ProbeBatchRecord> records;
    records.reserve(2);

    auto appendRecord = [&](const ProbeBatchEntry &entry,
                             const char *label) -> bool {
        std::vector<uint8_t> payload = serializeBatchValidated(
            mDeps.context, entry.iplBatch, entry.probeCount, label);
        if (payload.empty()) return false;
        ProbeBatchRecord rec;
        rec.purpose    = static_cast<uint32_t>(entry.purpose);
        rec.probeCount = static_cast<uint32_t>(entry.probeCount);
        rec.payload    = std::move(payload);
        records.push_back(std::move(rec));
        return true;
    };

    if (!appendRecord(reflEntry, "reflections")) {
        iplProbeBatchRelease(&reflEntry.iplBatch);
        if (havePathing) iplProbeBatchRelease(&pathEntry.iplBatch);
        return false;
    }
    if (havePathing) {
        if (!appendRecord(pathEntry, "pathing")) {
            iplProbeBatchRelease(&reflEntry.iplBatch);
            iplProbeBatchRelease(&pathEntry.iplBatch);
            return false;
        }
    }

    bool writeOk = writeProbeFile(outputPath, records);

    // Position sidecars (one per batch). Failure is non-fatal — overlay
    // just lacks positions until the next bake. Reflection batch carries
    // no per-probe radii (uniform spacing radius), so we pass an empty
    // vector and the writer omits the radius column.
    writePositionsSidecar(outputPath, ".reflections",
                          reflEntry.positions, reflEntry.radiiFt,
                          spacing, height);
    if (havePathing) {
        writePositionsSidecar(outputPath, ".pathing",
                              pathEntry.positions, pathEntry.radiiFt,
                              spacing, height);
    }

    // Bake done; release the batch handles. Disk is the source of truth
    // from here — the loader builds fresh batches from the file. (We
    // could keep these and skip the reload, but that would diverge the
    // bake-time and load-time code paths and bypass the disk validation.)
    iplProbeBatchRelease(&reflEntry.iplBatch);
    if (havePathing) iplProbeBatchRelease(&pathEntry.iplBatch);

    if (!writeOk) {
        LOG_ERROR("ProbeManager: failed to write probe file '%s'",
                  outputPath.c_str());
        return false;
    }

    AUDIO_LOG("Saved %d probes (refl=%d, pathing=%d) to '%s'\n",
              static_cast<int>(reflEntry.probeCount
                               + (havePathing ? pathEntry.probeCount : 0)),
              reflEntry.probeCount,
              havePathing ? pathEntry.probeCount : 0,
              outputPath.c_str());

    return true;
}

// ── Load ──────────────────────────────────────────────────────────────────

bool ProbeManager::loadProbes(const std::string &probePath,
                              IPLSimulator reflectionSimulator,
                              IPLSimulator pathingSimulator)
{
    if (!mDeps.context) {
        LOG_ERROR("ProbeManager: cannot load probes — no context");
        return false;
    }

    // Release any previously loaded batches (from both simulators).
    releaseBatches(reflectionSimulator, pathingSimulator);

    // Load and validate the multi-batch file.
    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> records;
    ProbeFileStatus status = loadProbeFile(probePath, hdr, records);

    if (status == ProbeFileStatus::FileNotFound) {
        LOG_INFO("ProbeManager: no probe file at '%s' — pathing/reverb disabled",
                 probePath.c_str());
        return false;
    }

    if (status == ProbeFileStatus::UnsupportedVersion) {
        LOG_ERROR("ProbeManager: probe file '%s' is from a previous format "
                  "version and cannot be migrated automatically. Delete it "
                  "and re-bake (debug console: `bake_probes`).",
                  probePath.c_str());
        return false;
    }

    if (status != ProbeFileStatus::Ok) {
        LOG_ERROR("ProbeManager: probe file '%s' failed validation: %s — "
                  "delete and re-bake",
                  probePath.c_str(), probeFileStatusString(status));
        return false;
    }

    // Reconstitute each batch.
    mBatches.clear();
    mBatches.reserve(records.size());
    for (const ProbeBatchRecord &rec : records) {
        ProbeBatchEntry entry;
        entry.purpose    = static_cast<ProbePurpose>(rec.purpose);
        entry.probeCount = static_cast<int>(rec.probeCount);

        IPLSerializedObjectSettings soSettings{};
        soSettings.data = const_cast<IPLbyte *>(rec.payload.data());
        soSettings.size = static_cast<IPLsize>(rec.payload.size());

        IPLSerializedObject serializedObject = nullptr;
        IPLerror err = iplSerializedObjectCreate(mDeps.context, &soSettings,
                                                  &serializedObject);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ProbeManager: iplSerializedObjectCreate failed (%d) "
                      "for batch %s in '%s'",
                      err, probePurposeString(entry.purpose), probePath.c_str());
            return false;
        }
        err = iplProbeBatchLoad(mDeps.context, serializedObject, &entry.iplBatch);
        iplSerializedObjectRelease(&serializedObject);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ProbeManager: iplProbeBatchLoad failed (%d) "
                      "for batch %s in '%s'",
                      err, probePurposeString(entry.purpose), probePath.c_str());
            return false;
        }
        iplProbeBatchCommit(entry.iplBatch);

        int loadedCount = iplProbeBatchGetNumProbes(entry.iplBatch);
        if (loadedCount != entry.probeCount) {
            LOG_ERROR("ProbeManager: batch %s probe count mismatch — record "
                      "says %d, Steam Audio loaded %d — discarding",
                      probePurposeString(entry.purpose),
                      entry.probeCount, loadedCount);
            iplProbeBatchRelease(&entry.iplBatch);
            return false;
        }

        // Detect which bake-data layers actually made it onto the batch.
        IPLBakedDataIdentifier reflId{};
        reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
        reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;
        entry.hasReflectionsData =
            (iplProbeBatchGetDataSize(entry.iplBatch, &reflId) > 0);

        IPLBakedDataIdentifier pathId{};
        pathId.type = IPL_BAKEDDATATYPE_PATHING;
        pathId.variation = IPL_BAKEDDATAVARIATION_DYNAMIC;
        entry.hasPathingData =
            (iplProbeBatchGetDataSize(entry.iplBatch, &pathId) > 0);

        // Load the positions sidecar for this batch (and per-probe radii
        // when present — only the pathing batch carries them currently).
        std::string suffix = (entry.purpose == ProbePurpose::Reflections)
                            ? ".reflections" : ".pathing";
        std::string posPath = probePath + suffix + ".positions.csv";
        entry.positions = readPositionsSidecar(posPath, entry.probeCount,
                                                &entry.radiiFt);
        if (static_cast<int>(entry.positions.size()) != entry.probeCount) {
            LOG_INFO("ProbeManager: probe positions sidecar '%s' count "
                     "mismatch (%zu vs %d) — overlay disabled for this batch",
                     posPath.c_str(), entry.positions.size(), entry.probeCount);
            entry.positions.clear();
            entry.radiiFt.clear();
        }

        mBatches.push_back(std::move(entry));
    }

    // Wire convenience pointers + totals.
    mReflectionsBatch = nullptr;
    mPathingBatch     = nullptr;
    mTotalProbeCount  = 0;
    for (auto &entry : mBatches) {
        mTotalProbeCount += entry.probeCount;
        if (entry.purpose == ProbePurpose::Reflections) {
            mReflectionsBatch = &entry;
        } else if (entry.purpose == ProbePurpose::Pathing) {
            mPathingBatch = &entry;
        }
    }

    // Attach every batch to both simulators. The reflection simulator
    // silently skips batches whose hasData(REFLECTIONS) is false (verified
    // in baked_reflection_simulator.cpp); the pathing simulator builds a
    // per-batch PathSimulator (simulation_manager.cpp:135) that is only
    // consumed when a source names that batch via
    // IPLSimulationInputs::pathingProbes.
    auto attachAll = [this](IPLSimulator sim, const char *label) {
        if (!sim) return;
        if (label == nullptr) label = "?";
        for (auto &entry : mBatches) {
            if (!entry.iplBatch) continue;
            iplSimulatorAddProbeBatch(sim, entry.iplBatch);
        }
        iplSimulatorCommit(sim);
        AUDIO_LOG("ProbeManager: attached %zu batch(es) to %s simulator\n",
                  mBatches.size(), label);
    };

    if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();
    if (mDeps.waitForPathingThread)    mDeps.waitForPathingThread();

    attachAll(reflectionSimulator, "reflection");
    attachAll(pathingSimulator,    "pathing");

    AUDIO_LOG("ProbeManager: loaded %d probes from '%s' "
              "(refl_batch=%d w/refl=%s, pathing_batch=%d w/pathing=%s)\n",
              mTotalProbeCount, probePath.c_str(),
              mReflectionsBatch ? mReflectionsBatch->probeCount : 0,
              (mReflectionsBatch && mReflectionsBatch->hasReflectionsData)
                  ? "yes" : "no",
              mPathingBatch ? mPathingBatch->probeCount : 0,
              (mPathingBatch && mPathingBatch->hasPathingData) ? "yes" : "no");
    return true;
}

// ── Release ───────────────────────────────────────────────────────────────

void ProbeManager::releaseEntry(ProbeBatchEntry &entry,
                                IPLSimulator reflectionSimulator,
                                IPLSimulator pathingSimulator)
{
    if (!entry.iplBatch) return;

    if (reflectionSimulator) {
        iplSimulatorRemoveProbeBatch(reflectionSimulator, entry.iplBatch);
    }
    if (pathingSimulator) {
        iplSimulatorRemoveProbeBatch(pathingSimulator, entry.iplBatch);
    }
    iplProbeBatchRelease(&entry.iplBatch);
    entry.iplBatch = nullptr;
    entry.probeCount = 0;
    entry.hasReflectionsData = false;
    entry.hasPathingData = false;
    entry.positions.clear();
    entry.radiiFt.clear();
}

void ProbeManager::releaseBatches(IPLSimulator reflectionSimulator,
                                  IPLSimulator pathingSimulator)
{
    if (mBatches.empty()) return;

    if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();
    if (mDeps.waitForPathingThread)    mDeps.waitForPathingThread();

    for (auto &entry : mBatches) {
        releaseEntry(entry, reflectionSimulator, pathingSimulator);
    }
    if (reflectionSimulator) iplSimulatorCommit(reflectionSimulator);
    if (pathingSimulator)    iplSimulatorCommit(pathingSimulator);

    mBatches.clear();
    mReflectionsBatch = nullptr;
    mPathingBatch = nullptr;
    mTotalProbeCount = 0;
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
