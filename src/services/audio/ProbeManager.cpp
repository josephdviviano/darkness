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

// ── Shared placement passes (dry-run / live bake) ────────────────────────
//
// The reflection-batch placement pipeline (floor grid → elevation tier →
// emitter anchors → global dedup) used to live inline in
// `bakeReflectionBatch`. It now lives here so the dry-run probe-count
// tool (`darknessHeadless probe_plan`, Capability A in
// PLAN.PROBE_DEBUG_TOOLING) can call the same code path and get
// byte-identical probe counts to the live bake — no rounding divergence.
//
// Same for `computePathingPlacements` below: it was the candidate-filter
// loop at the top of `bakePathingBatch` (pre-IPL-commit). The dedup pass
// that AudioService runs on `pathingCandidates` BEFORE handing them to
// ProbeManager is intentionally not part of this seam — it lives in
// `AudioService::prepareProbeBakeParams` (which both `bakeProbes` and
// `computeProbePlan` invoke).

bool ProbeManager::computeReflectionPlacements(IPLScene scene,
                                               const ProbeBakeParams &params,
                                               float spacing,
                                               float height,
                                               ProbeBakePlan &plan)
{
    AUDIO_LOG("Reflection placements: spacing=%.1f height=%.1f "
              "(min_wall_clearance=%.1fft, filter=%s, scheme=FLOOR_POLY+UNIFORMFLOOR)\n",
              spacing, height, params.minWallClearanceFt,
              params.probeFilter ? "enabled" : "disabled");

    // Two-pass floor placement: FLOOR_POLY first (semantically meaningful,
    // wins dedup), UNIFORMFLOOR second (fills in big open cells where
    // FLOOR_POLY emits a single centroid for a 200x200ft courtyard).
    //
    // FLOOR_POLY: AcousticSceneData ships one candidate per upward-facing
    // BSP cell polygon (plane.normal.z > 0.5); AudioService stashes them
    // at scene-build time and forwards them in params.floorProbeCandidates.
    // Guarantees ground coverage in every BSP cell that has a floor poly
    // — fixes miss14, where UNIFORMFLOOR's single-OBB raycasts returned
    // zero (its OBB top sat in solid rock above the deep tunnels). See
    // PLAN.AUDIO_PROFILING.md scheme comparison.
    //
    // UNIFORMFLOOR: Steam Audio's OBB-grid raycaster. Cheap on flat
    // single-level layouts, undersamples big rooms only if FLOOR_POLY
    // didn't pre-cover them. Runs second so dedup collapses redundant
    // overlaps in FLOOR_POLY's favor.
    if (const char *objEnv = std::getenv("DARKNESS_DUMP_ACOUSTIC_OBJ")) {
        iplSceneSaveOBJ(scene, const_cast<IPLstring>(objEnv));
        std::fprintf(stderr,
            "[PROBE_PLAN] DARKNESS_DUMP_ACOUSTIC_OBJ — wrote scene to %s\n",
            objEnv);
    }

    const auto &candidates = params.floorProbeCandidates;
    AUDIO_LOG("Reflection scene bounds engine-ft: (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n"
              "FLOOR_POLY candidates: %zu (one per upward-facing BSP poly)\n",
              params.sceneMin.x, params.sceneMin.y, params.sceneMin.z,
              params.sceneMax.x, params.sceneMax.y, params.sceneMax.z,
              candidates.size());

    if (candidates.empty()) {
        // Empty list means the WR walker never populated floor candidates
        // (likely an older code path bypassed buildAcousticScene's data
        // ingestion, or the mission has zero BSP cells).  Loud failure
        // — silently producing a probe-less reflection batch would mask
        // a real bug.
        std::fprintf(stderr,
            "[FALLBACK] ProbeManager: no floor-poly candidates supplied "
            "by AcousticSceneData. Reflection batch will be empty. "
            "Check that the WR-walk loop in DarknessRender/DarknessHeadless "
            "populates AcousticSceneData::floorProbeCandidates.\n");
        return false;
    }

    std::vector<Vector3> &positions = plan.reflectionPositions;
    positions.clear();
    positions.reserve(candidates.size());

    // Per-probe filtered add — drops in-wall / wall-adjacent / unreachable
    // candidates BEFORE baking, so the .probes file never contains bad
    // probes. Placement passes APPEND to positions; the IPL batch is
    // populated in a single final loop after a global dedup pass filters
    // cross-pass overlaps.
    int rejectedFloor = 0;
    int nudgedFloor   = 0;
    const Vector3 heightOffset(0.0f, 0.0f, height);
    for (const Vector3 &floorPos : candidates) {
        // The candidate is the centroid of the floor polygon at floor
        // level; place the probe `height` ft above it along world Z+.
        // Matches the original UNIFORMFLOOR convention (probes placed
        // along IPL +Y above the hit point = engine +Z above the floor).
        PlaceResult pr = tryPlaceProbe(floorPos + heightOffset, params.probeFilter);
        if (!pr.ok) { ++rejectedFloor; continue; }
        if (pr.iters > 0) ++nudgedFloor;
        positions.push_back(pr.pos);
    }

    const int polyKept     = static_cast<int>(positions.size());
    const int numCandidates = static_cast<int>(candidates.size());
    AUDIO_LOG("FLOOR_POLY probes: %d kept (%d nudged), %d rejected by filter "
              "(%d candidates)\n",
              polyKept, nudgedFloor, rejectedFloor, numCandidates);

    // ── UNIFORMFLOOR pass (OBB-grid fill) ──────────────────────────────
    // Runs second so the global dedup at the end of this function favors
    // FLOOR_POLY centroids over UNIFORMFLOOR grid samples that overlap.
    // The pass is best-effort: a 0-candidate return (miss14-style scene)
    // is non-fatal because FLOOR_POLY already provided per-cell coverage.
    int unifloorKept = 0;
    {
        IPLProbeArray probeArray = nullptr;
        IPLerror err = iplProbeArrayCreate(mDeps.context, &probeArray);
        if (err != IPL_STATUS_SUCCESS) {
            // Non-fatal; FLOOR_POLY pass above already populated positions.
            std::fprintf(stderr,
                "[FALLBACK] ProbeManager: iplProbeArrayCreate failed (%d) — "
                "UNIFORMFLOOR fill skipped, relying on FLOOR_POLY pass\n", err);
        } else {
            // UNIFORMFLOOR transform maps the UNIT CUBE [0,1]³ into world
            // space.  OBB must be in IPL Y-up space — raycasts go along
            // IPL -Y to find floors. Expanded by one spacing margin so
            // probes cover near-edge areas.
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

            IPLProbeGenerationParams genParams{};
            genParams.type = IPL_PROBEGENERATIONTYPE_UNIFORMFLOOR;
            genParams.spacing = spacing * kFeetToMeters;
            genParams.height  = height  * kFeetToMeters;
            genParams.transform = transform;

            iplProbeArrayGenerateProbes(probeArray, scene, &genParams);
            const int numUniCands = iplProbeArrayGetNumProbes(probeArray);

            int unifloorRejected = 0;
            int unifloorNudged   = 0;
            for (int i = 0; i < numUniCands; ++i) {
                IPLSphere s = iplProbeArrayGetProbe(probeArray, i);
                Vector3 enginePos = iplToEnginePos(s.center);
                PlaceResult pr = tryPlaceProbe(enginePos, params.probeFilter);
                if (!pr.ok) { ++unifloorRejected; continue; }
                if (pr.iters > 0) ++unifloorNudged;
                positions.push_back(pr.pos);
                ++unifloorKept;
            }
            iplProbeArrayRelease(&probeArray);

            AUDIO_LOG("UNIFORMFLOOR probes: %d kept (%d nudged), %d rejected "
                      "by filter (%d candidates, spacing=%.1f ft)\n",
                      unifloorKept, unifloorNudged, unifloorRejected,
                      numUniCands, spacing);
        }
    }

    const int floorKept = static_cast<int>(positions.size());
    plan.floorKept = floorKept;

    if (floorKept == 0) {
        // BOTH passes produced zero usable probes. FLOOR_POLY can only
        // fail this way if the WR walker didn't populate candidates AND
        // UNIFORMFLOOR also misfired (miss14 + zero floor polys = no
        // mission can survive this). Fail loudly rather than writing an
        // empty .probes file that breaks reverb.
        LOG_ERROR("ProbeManager: both FLOOR_POLY (%d candidates, %d "
                  "filter-rejected) and UNIFORMFLOOR (%d kept) produced "
                  "zero probes — refusing to bake an empty batch. Check "
                  "audio.probes.min_wall_clearance_ft and "
                  "AcousticSceneData::floorProbeCandidates population.",
                  numCandidates, rejectedFloor, unifloorKept);
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
        plan.elevationKept = addedElev;
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
        plan.emitterKept = emitterAdded;

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
    //
    // ROOM-AWARE when params.probeRoomLookup is set: probes only collapse
    // against other probes in the SAME room. This protects per-room
    // coverage when adjacent rooms have floor-poly centroids less than
    // dedupRadius apart across a thin wall — the room-agnostic legacy
    // path silently stripped entire small rooms on FLOOR_POLY missions
    // (the playtest showed lstRoom=13 with 30% same-room sentinels
    // because adjacent rooms ate its probes via cross-room dedup).
    if (params.globalDedupRadiusFt > 0.0f && positions.size() > 1) {
        const float dedupRadiusSq = params.globalDedupRadiusFt
                                  * params.globalDedupRadiusFt;
        const bool roomAware = static_cast<bool>(params.probeRoomLookup);
        std::vector<Vector3> kept;
        std::vector<int>     keptRoom;  // parallel; -1 in non-room-aware mode
        kept.reserve(positions.size());
        keptRoom.reserve(positions.size());
        int globalDeduped       = 0;
        int crossRoomPreserved  = 0;
        for (const Vector3 &p : positions) {
            const int probeRoom = roomAware ? params.probeRoomLookup(p) : -1;
            bool tooClose = false;
            for (size_t i = 0; i < kept.size(); ++i) {
                Vector3 d = kept[i] - p;
                if (glm::dot(d, d) >= dedupRadiusSq) continue;
                if (roomAware && keptRoom[i] != probeRoom) {
                    // Same distance but different room — these cover
                    // distinct acoustic spaces. Preserve both.
                    ++crossRoomPreserved;
                    continue;
                }
                tooClose = true;
                break;
            }
            if (tooClose) { ++globalDeduped; continue; }
            kept.push_back(p);
            keptRoom.push_back(probeRoom);
        }
        if (globalDeduped > 0 || crossRoomPreserved > 0) {
            AUDIO_LOG("Global dedup: kept %zu / %zu probes "
                      "(%d deduped at %.1f ft, %d cross-room preserved%s)\n",
                      kept.size(), positions.size(),
                      globalDeduped, params.globalDedupRadiusFt,
                      crossRoomPreserved,
                      roomAware ? "" : " — room lookup DISABLED");
        }
        plan.globalDeduped = globalDeduped;
        positions = std::move(kept);
    }

    return true;
}

// ── Pathing placement (filter only — dedup lives in AudioService) ────────

void ProbeManager::computePathingPlacements(const ProbeBakeParams &params,
                                            ProbeBakePlan &plan)
{
    plan.pathingKept.clear();
    plan.pathingPerPurpose.clear();

    if (params.pathingCandidates.empty()) {
        AUDIO_LOG("Pathing placements: no candidates supplied\n");
        return;
    }

    plan.pathingKept.reserve(params.pathingCandidates.size());

    int rejected = 0;
    int nudged   = 0;
    for (const auto &cand : params.pathingCandidates) {
        PlaceResult pr = tryPlaceProbe(cand.position, params.pathingProbeFilter);
        if (!pr.ok) { ++rejected; continue; }
        if (pr.iters > 0) ++nudged;
        PathingProbeCandidate kept = cand;
        kept.position = pr.pos;
        plan.pathingKept.push_back(kept);
        plan.pathingPerPurpose[kept.purpose] += 1;
    }
    AUDIO_LOG("Pathing placements: kept %zu probes (%d nudged, %d rejected) "
              "from %zu candidates\n",
              plan.pathingKept.size(), nudged, rejected,
              params.pathingCandidates.size());
}

// ── Dry-run plan ──────────────────────────────────────────────────────────

bool ProbeManager::computeBakePlan(IPLScene scene,
                                   const ProbeBakeParams &params,
                                   ProbeBakePlan &out)
{
    if (!mDeps.context || !scene) {
        LOG_ERROR("ProbeManager::computeBakePlan: no acoustic scene");
        return false;
    }

    // NOTE: we intentionally do NOT zero `out` here. The caller
    // (AudioService::computeProbePlan) may have already populated
    // dedup counters via prepareProbeBakeParams before invoking
    // this function. The reflection / pathing placement passes below
    // only write to the fields they own (reflectionPositions, floor/
    // elevation/emitter/globalDeduped, pathingKept, pathingPerPurpose)
    // so the dedup counters survive intact.

    float spacing = (params.spacingFtOverride > 0.0f) ? params.spacingFtOverride : mProbeSpacingFt;
    float height  = (params.heightFtOverride  > 0.0f) ? params.heightFtOverride  : mProbeHeightFt;

    if (!computeReflectionPlacements(scene, params, spacing, height, out)) {
        return false;
    }

    if (params.bakePathingBatch) {
        computePathingPlacements(params, out);
    }

    return true;
}

// ── Reflection batch bake (IPL commit + bake — wraps placement pass) ─────

bool ProbeManager::bakeReflectionBatch(IPLScene scene,
                                       const ProbeBakeParams &params,
                                       float spacing,
                                       float height,
                                       std::atomic<float> *progress,
                                       const std::string &outputPath,
                                       ProbeBatchEntry &outEntry)
{
    // Run the shared placement pass into a temporary plan, then commit
    // the surviving positions into the IPL batch and bake IRs. Same
    // code path as the dry-run (`computeBakePlan`) so the bake's probe
    // count exactly matches the dry-run's predicted count.
    ProbeBakePlan plan;
    if (!computeReflectionPlacements(scene, params, spacing, height, plan)) {
        return false;
    }

    outEntry.positions = std::move(plan.reflectionPositions);
    std::vector<Vector3> &positions = outEntry.positions;

    IPLProbeBatch probeBatch = nullptr;
    IPLerror err = iplProbeBatchCreate(mDeps.context, &probeBatch);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: iplProbeBatchCreate failed (%d)", err);
        return false;
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

    // Run the shared filter pass (same code as the dry-run
    // `computeBakePlan`) so bake and dry-run produce identical
    // post-filter survivor counts.
    ProbeBakePlan plan;
    computePathingPlacements(params, plan);

    std::vector<Vector3> &positions = outEntry.positions;
    std::vector<float>   &radii     = outEntry.radiiFt;
    positions.clear();
    radii.clear();
    positions.reserve(plan.pathingKept.size());
    radii.reserve(plan.pathingKept.size());
    for (const auto &k : plan.pathingKept) {
        positions.push_back(k.position);
        radii.push_back(k.radiusFt);
    }

    if (positions.empty()) {
        LOG_ERROR("ProbeManager: filter rejected every pathing candidate — "
                  "refusing to bake an empty pathing batch");
        return false;
    }

    IPLProbeBatch probeBatch = nullptr;
    IPLerror err = iplProbeBatchCreate(mDeps.context, &probeBatch);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("ProbeManager: pathing batch iplProbeBatchCreate failed (%d)",
                  err);
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
    // Visibility sampling count. Single source of truth is the
    // SteamAudioPathing.h profile constants (kPathingVisSamplesShip /
    // kPathingVisSamplesDev, selected by --bake-quality dev);
    // AudioService fills params.pathingNumSamples from the SAME
    // selection that configures the runtime simulator's numVisSamples,
    // so bake and runtime cannot diverge within a run. Cross-run
    // divergence (cache baked under the other profile) is caught by
    // the .probes v3 header record + the loader's mismatch re-bake.
    // Rays per probe-pair test = numSamples².
    bakeParams.numSamples = params.pathingNumSamples;
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
    // bake will even consider building a visibility edge — pairs beyond
    // it are culled BEFORE the numSamples² ray test
    // (`ProbeVisibilityTester::areProbesTooFar`, path_visibility.cpp:95,
    // called at the top of the pair loop, path_visibility.cpp:157), so
    // this is the dominant bake-cost lever on sprawling missions.
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
    // have reached them.
    //
    // The lesson from that bug is about pathRange and about SILENCE,
    // not about capping visRange: the two knobs are independent
    // (verified in Steam Audio's source — visRange gates single EDGES,
    // pathRange gates total ROUTE length), and the ray cost lives
    // entirely on visRange. So they are now split:
    //
    //   • visRange = the SINGLE-EDGE cap, derived from the mission's
    //     own ROOM_DB (params.pathingVisRangeFt: max intra-room
    //     portal-to-portal span / room diameter × 1.5 margin, clamped
    //     to [150, 400] ft — see AudioService::prepareProbeBakeParams).
    //     The longest meaningful single hop is a room-scale distance:
    //     any two probes further apart than the largest room either
    //     have geometry between them (edge would fail the ray test
    //     anyway — we're paying 256 rays to discover "no") or are
    //     connected through intermediate room/portal probes (multi-hop
    //     covers them). Runtime per-voice `inputs.visRange` is clamped
    //     to this cap with a loud [FALLBACK] (AudioService loopStep),
    //     so nothing exceeds the baked graph silently.
    //
    //     NOTE (upstream behaviour): the bake's range cull is
    //     HORIZONTAL-ONLY. api_baking.cpp:110 forces
    //     asymmetricVisRange=true with down=(0,-1,0) (IPL Y-up), and
    //     areProbesTooFar then projects out the vertical component
    //     before comparing against visRange. Fine for Thief footprints
    //     (vertical spans ≪ room spans), but a cap derived from
    //     horizontal room sizes must not be "optimized" down to
    //     exclude tall shafts — it doesn't bound vertical distance at
    //     all.
    //
    //   • pathRange = the MULTI-HOP total-route cap. Keeps the
    //     generous whole-level value (scene AABB diagonal × 1.5,
    //     ceiling 5000 ft ≈ 1500 m — Valve's Unity convention of a
    //     route budget that never truncates) so long cross-mission
    //     routes still exist as chains of room-scale edges. This is
    //     the knob the documented truncation bug was really about;
    //     it stays uncapped-by-room-size on purpose.
    const Vector3 sceneSpan = params.sceneMax - params.sceneMin;
    const float sceneDiagonalFt = std::sqrt(sceneSpan.x * sceneSpan.x
                                          + sceneSpan.y * sceneSpan.y
                                          + sceneSpan.z * sceneSpan.z);
    constexpr float kBakeRangeMarginMul = 1.5f;
    constexpr float kBakeRangeCeilingFt = 5000.0f;  // ~1500 m — Valve-ish ceiling
    const float pathRangeFt = std::min(
        std::max(sceneDiagonalFt * kBakeRangeMarginMul,
                 params.propagationMaxDist * kBakeRangeMarginMul),
        kBakeRangeCeilingFt);
    float visRangeFt = params.pathingVisRangeFt;
    if (visRangeFt <= 0.0f) {
        // No derived cap supplied (caller had no RoomService-derived
        // span). Whole-level visRange is the safe-but-slow pre-cap
        // behaviour; announce loudly because the bake is about to pay
        // the full O(pairs) ray bill the cap exists to avoid.
        std::fprintf(stderr,
            "[FALLBACK] bakePathingBatch: no derived pathing visRange "
            "cap supplied (pathingVisRangeFt=%.1f) — falling back to "
            "whole-level range %.0f ft; bake will be SLOW\n",
            params.pathingVisRangeFt, pathRangeFt);
        visRangeFt = pathRangeFt;
    }
    bakeParams.visRange  = visRangeFt * kFeetToMeters;
    bakeParams.pathRange = pathRangeFt * kFeetToMeters;
    AUDIO_LOG("Pathing bake range: visRange=%.0f ft (%s) | pathRange="
              "%.0f ft (sceneDiag=%.0f ft × %.1fx, ceiling=%.0f ft) | "
              "numSamples=%d (%d rays/pair)\n",
              visRangeFt,
              (params.pathingVisRangeFt > 0.0f) ? "ROOM_DB-derived cap"
                                                : "whole-level fallback",
              pathRangeFt, sceneDiagonalFt, kBakeRangeMarginMul,
              kBakeRangeCeilingFt, bakeParams.numSamples,
              bakeParams.numSamples * bakeParams.numSamples);
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
    // Effective bake profile (post-fallback) — recorded into the .probes
    // v3 header by bakeProbes so future loads can verify runtime/bake
    // agreement.
    outEntry.bakedVisRangeFt    = visRangeFt;
    outEntry.bakedNumSamples    = bakeParams.numSamples;
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

    // Reflection section: fresh IR bake, or — for a pathing-only re-bake
    // (params.reuseLoadedReflectionsBatch: --force-pathing-bake / the
    // automatic v3 header-mismatch re-bake) — carry the CURRENTLY LOADED
    // reflection batch forward into the output file unchanged. The
    // reflection IR bake is the expensive half and nothing about it
    // depends on the pathing profile, so re-serializing the loaded batch
    // is exact. Precondition: a loaded batch with IR data; when absent
    // we announce and fall back to the full bake rather than silently
    // writing a reflection-less file.
    bool carryForwardRefl = false;
    if (params.reuseLoadedReflectionsBatch) {
        if (mReflectionsBatch && mReflectionsBatch->iplBatch
            && mReflectionsBatch->hasReflectionsData) {
            carryForwardRefl = true;
        } else {
            std::fprintf(stderr,
                "[FALLBACK] bakeProbes: pathing-only re-bake requested "
                "but no loaded reflection batch with IR data is available "
                "— running the FULL (multi-minute) reflection bake "
                "instead\n");
        }
    }

    ProbeBatchEntry reflEntry;
    if (carryForwardRefl) {
        // Serialization is a read-only walk of the batch, but keep the
        // same drain-the-workers discipline as every other access to a
        // live (simulator-attached) batch.
        if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();
        if (mDeps.waitForPathingThread)    mDeps.waitForPathingThread();
        reflEntry.purpose            = ProbePurpose::Reflections;
        reflEntry.iplBatch           = mReflectionsBatch->iplBatch; // BORROWED — released via mBatches, not here
        reflEntry.probeCount         = mReflectionsBatch->probeCount;
        reflEntry.hasReflectionsData = true;
        reflEntry.positions          = mReflectionsBatch->positions;
        reflEntry.radiiFt            = mReflectionsBatch->radiiFt;
        AUDIO_LOG("bakeProbes: pathing-only re-bake — carrying forward "
                  "the loaded reflection batch (%d probes) unchanged\n",
                  reflEntry.probeCount);
    } else if (!bakeReflectionBatch(scene, params, spacing, height,
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

    // Release helper that respects carry-forward: a borrowed reflection
    // handle belongs to mBatches (released by the loader's
    // releaseBatches on the post-bake reload), never here.
    auto releaseOwned = [&]() {
        if (reflEntry.iplBatch && !carryForwardRefl)
            iplProbeBatchRelease(&reflEntry.iplBatch);
        if (havePathing && pathEntry.iplBatch)
            iplProbeBatchRelease(&pathEntry.iplBatch);
    };

    if (!appendRecord(reflEntry, "reflections")) {
        releaseOwned();
        return false;
    }
    if (havePathing) {
        if (!appendRecord(pathEntry, "pathing")) {
            releaseOwned();
            return false;
        }
    }

    // v3 header records the pathing bake profile (0s when no pathing
    // section) so future loads can verify the cache against the active
    // runtime profile.
    const float    hdrVisRangeFt = havePathing ? pathEntry.bakedVisRangeFt : 0.0f;
    const uint32_t hdrNumSamples = havePathing
        ? static_cast<uint32_t>(pathEntry.bakedNumSamples) : 0u;
    bool writeOk = writeProbeFile(outputPath, records,
                                  hdrVisRangeFt, hdrNumSamples);

    // Position sidecars (one per batch). Failure is non-fatal — overlay
    // just lacks positions until the next bake. Reflection batch carries
    // no per-probe radii (uniform spacing radius), so we pass an empty
    // vector and the writer omits the radius column. On carry-forward
    // the reflection sidecar on disk is already correct (and the loaded
    // positions mirror may be empty if that sidecar was missing), so we
    // leave it untouched.
    if (!carryForwardRefl) {
        writePositionsSidecar(outputPath, ".reflections",
                              reflEntry.positions, reflEntry.radiiFt,
                              spacing, height);
    }
    if (havePathing) {
        writePositionsSidecar(outputPath, ".pathing",
                              pathEntry.positions, pathEntry.radiiFt,
                              spacing, height);
    }

    // Bake done; release the batch handles we own. Disk is the source of
    // truth from here — the loader builds fresh batches from the file.
    // (We could keep these and skip the reload, but that would diverge
    // the bake-time and load-time code paths and bypass the disk
    // validation.)
    releaseOwned();

    if (!writeOk) {
        LOG_ERROR("ProbeManager: failed to write probe file '%s'",
                  outputPath.c_str());
        return false;
    }

    // Mirror the just-written bake profile so the runtime guard /
    // mismatch check work immediately after a bake, before (or without)
    // the post-bake reload.
    mBakedPathingVisRangeFt = hdrVisRangeFt;
    mBakedPathingNumSamples = static_cast<int>(hdrNumSamples);

    const int reflProbeCount = reflEntry.probeCount;
    AUDIO_LOG("Saved %d probes (refl=%d, pathing=%d) to '%s'\n",
              static_cast<int>(reflProbeCount
                               + (havePathing ? pathEntry.probeCount : 0)),
              reflProbeCount,
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
                  "version (v3 added the pathing bake-profile record) and "
                  "cannot be migrated automatically. Delete it and re-bake "
                  "(debug console: `bake_probes`) — the startup flow will "
                  "also auto-re-bake when no loadable file is present.",
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

    // Record the pathing bake profile from the v3 header. Consumers:
    // the runtime per-voice visRange clamp and the AudioService-side
    // numSamples mismatch check (both read the getBakedPathing*
    // accessors).
    mBakedPathingVisRangeFt = hdr.bakedPathingVisRangeFt;
    mBakedPathingNumSamples = static_cast<int>(hdr.bakedPathingNumSamples);

    // Attach batches to the simulators that can actually consume them.
    //
    // The reflection simulator must ONLY see batches carrying baked
    // reflections data. An earlier version attached every batch on the
    // belief that Steam Audio "silently skips batches whose
    // hasData(REFLECTIONS) is false" — that skip exists at the batch
    // level (baked_reflection_simulator.cpp, lookupEnergyField), but the
    // probe NEIGHBORHOOD passed into each batch's evaluateEnergyField
    // still spans every attached batch, and its per-probe guard
    // (`batches[i]->hasData(id) && &batch[id] != this`) falls through
    // for batches with NO reflections data. Their probe indices are then
    // applied to the reflections batch's own arrays
    // (BakedReflectionsData::lookupEnergyField has no bounds check):
    // wrong probes' energy fields where the index happens to be in
    // range, and an out-of-bounds read → garbage EnergyField pointer →
    // EXC_BAD_ACCESS on the reflection worker where it isn't. MISS2 was
    // the first mission whose pathing batch (949 probes) outnumbered its
    // reflections batch (626) and crashed within seconds of touring;
    // missions with pathing < reflections (MISS6: 630 < 833) never
    // crashed but silently blended unrelated probes' reverb.
    //
    // The pathing simulator keeps every batch: it builds a per-batch
    // PathSimulator (simulation_manager.cpp:135) that is only consumed
    // when a source names that batch via
    // IPLSimulationInputs::pathingProbes, so foreign batches are inert
    // there.
    auto attachMatching = [this](IPLSimulator sim, const char *label,
                                  auto &&wants) {
        if (!sim) return;
        if (label == nullptr) label = "?";
        size_t attached = 0;
        for (auto &entry : mBatches) {
            if (!entry.iplBatch) continue;
            if (!wants(entry)) continue;
            iplSimulatorAddProbeBatch(sim, entry.iplBatch);
            ++attached;
        }
        iplSimulatorCommit(sim);
        AUDIO_LOG("ProbeManager: attached %zu of %zu batch(es) to %s "
                  "simulator\n", attached, mBatches.size(), label);
    };

    if (mDeps.waitForReflectionThread) mDeps.waitForReflectionThread();
    if (mDeps.waitForPathingThread)    mDeps.waitForPathingThread();

    attachMatching(reflectionSimulator, "reflection",
                   [](const ProbeBatchEntry &e) {
                       return e.hasReflectionsData;
                   });
    attachMatching(pathingSimulator, "pathing",
                   [](const ProbeBatchEntry &) { return true; });

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
    mBakedPathingVisRangeFt = 0.0f;
    mBakedPathingNumSamples = 0;

    // The adjacency was built against the now-released pathing batch.
    // Leaving it in place would surface stale edges over a new mission's
    // probe set, so drop it. AudioService rebuilds after the next
    // loadProbes / bakeProbes.
    clearPathingAdjacency();
}

// ── Pathing adjacency build (Capability C debug viz) ─────────────────────
//
// NOT a claim that Steam Audio visited this specific edge for any
// specific voice. The adjacency is computed by Darkness using the
// same raycast algorithm Steam Audio's pathing baker uses
// (IPLPathBakeParams::visRange + IPLPathBakeParams::numSamples,
// `phonon.h` ~lines 3563-3584). PARTIAL match — the visRange distance
// gate matches exactly, but the visibility test itself does NOT.
//
// Steam Audio scatters `numSamples`² rays inside a `visRadius` sphere
// around each probe (path_visibility.cpp) and counts a pair visible
// if the fraction of unblocked rays meets `visThreshold`. Darkness's
// adjacency runs ONE center-to-center ray per pair — Steam Audio's
// degenerate `numSamples=1, radius=0` fast path (path_visibility.cpp:
// ~88-90). Expect occasional false-positive edges (thin walls between
// probe centers that Steam Audio would catch with sphere sampling)
// and false-negative edges (tight clearances around centers that
// Steam Audio averages out). Acceptable for a neighborhood-activity
// heatmap; would need to be promoted to numSamples² raycasts (~16× the
// O(N²) cost) for bit-exact agreement with the bake.
//
// Layer-2 coloring (done in the renderer / AudioService::
// getPathingEdgeViz) is a neighborhood-activity heatmap: "voices near
// this edge's endpoints are perceiving the listener with this EQ
// quality." It does NOT enumerate Steam Audio's actual visited-probe
// set per voice — the public C API does not expose that data, and
// inferring it via BFS was rejected (PLAN.PROBE_DEBUG_TOOLING.md,
// Capability C) due to divergence risk.
void ProbeManager::buildPathingAdjacency(
    float visRangeFt,
    int   numVisSamples,
    const PathingAdjacencyRaycaster &raycast)
{
    // Reset cache up front so a partial / failed build doesn't leave
    // stale edges visible from a prior run.
    mPathingAdjacency = {};
    mPathingAdjacency.visRangeFt    = visRangeFt;
    mPathingAdjacency.numVisSamples = numVisSamples;

    if (!mPathingBatch || mPathingBatch->positions.empty()) {
        // Nothing to test — leave built=false; the renderer's HUD path
        // surfaces "NO PATHING BATCH". No log: this is the normal state
        // for missions without a pathing bake.
        return;
    }
    if (!raycast) {
        // Hard fallback — every pair would default to visible without a
        // raycaster, which would produce a meaningless fully-connected
        // graph. Loud per feedback_no_silent_fallbacks: tell the user
        // why the overlay is dark.
        std::fprintf(stderr,
            "[VIZ_FALLBACK] ProbeManager::buildPathingAdjacency: no "
            "raycaster supplied — adjacency cannot be computed; "
            "show_pathing_graph will be empty\n");
        return;
    }

    const auto &positions = mPathingBatch->positions;
    const int N = static_cast<int>(positions.size());
    const float visRangeSqFt = visRangeFt * visRangeFt;

    auto t0 = std::chrono::steady_clock::now();
    int edges = 0;
    int rangeRejects = 0;
    mPathingAdjacency.edges.reserve(static_cast<size_t>(N) * 4);

    // O(N²) probe-pair pass. At N=250 this is ~31000 unique pairs;
    // even with one raycast per pair this completes in a few hundred
    // ms on a debug build. Single-shot at level-load; never per-frame.
    for (int i = 0; i < N; ++i) {
        const Vector3 &a = positions[i];
        for (int j = i + 1; j < N; ++j) {
            const Vector3 &b = positions[j];
            Vector3 d = b - a;
            float distSq = d.x*d.x + d.y*d.y + d.z*d.z;
            if (distSq > visRangeSqFt) {
                ++rangeRejects;
                continue;
            }
            // Center-to-center ray test against the acoustic mesh.
            // `raycast` returns true on a hit (segment blocked).
            // Mirrors Steam Audio's `scene.isOccluded` semantic.
            if (raycast(a, b)) {
                continue;  // blocked → no edge
            }
            mPathingAdjacency.edges.push_back({i, j});
            ++edges;
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    mPathingAdjacency.built   = true;
    mPathingAdjacency.buildMs = ms;

    // Single-line diagnostic that lets future debugging cross-check
    // edge count against Steam Audio's reported bake count (if a future
    // patch exposes it). Edge count is the principal cross-check.
    // numVisSamples reported as `requested=N (actually 1 center ray)`
    // so the log doesn't pretend we used Steam Audio's full
    // sphere-sample protocol. See comment block above.
    std::fprintf(stderr,
        "[ADJACENCY_BUILD] %d probes, %d edges, %d range-rejects, "
        "%.1f ms (visRange=%.1f ft, numVisSamples=requested=%d "
        "actually=1 center ray)\n",
        N, edges, rangeRejects, ms, visRangeFt, numVisSamples);
}

// ── T2.4 IR cache stats dump ─────────────────────────────────────────────
//
// Self-throttled to ~1 Hz. Caller passes the worker pool's active-slot
// snapshot (active vs MAX) and the eviction count from
// `ConvolutionWorker::slotEvictionsTotal`. We read+reset our own hit/miss
// counters so each emission window is independent.

void ProbeManager::pollPerfPeriodic(int activeSlots, int maxSlots,
                                    uint64_t evictionsSinceLast)
{
    static std::chrono::steady_clock::time_point sLast{};
    auto now = std::chrono::steady_clock::now();
    if (sLast.time_since_epoch().count() != 0) {
        auto ms = std::chrono::duration<double, std::milli>(now - sLast).count();
        if (ms < 1000.0) return;
    }
    sLast = now;

    uint64_t hits   = mCacheHits.exchange(0, std::memory_order_relaxed);
    uint64_t misses = mCacheMisses.exchange(0, std::memory_order_relaxed);
    uint64_t total  = hits + misses;
    double rate     = (total > 0)
                    ? static_cast<double>(hits) / static_cast<double>(total)
                    : 0.0;
    AUDIO_LOG(
        "[PERF refl_cache] hits=%llu misses=%llu hitRate=%.3f"
        " evictions=%llu activeSlots=%d/%d\n",
        static_cast<unsigned long long>(hits),
        static_cast<unsigned long long>(misses),
        rate,
        static_cast<unsigned long long>(evictionsSinceLast),
        activeSlots, maxSlots);
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
