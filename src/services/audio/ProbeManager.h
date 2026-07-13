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

#ifndef __PROBE_MANAGER_H
#define __PROBE_MANAGER_H

/// @file ProbeManager.h
/// Acoustic probe baking, loading, saving, and in-memory storage. Owns one
/// or more Steam Audio probe batches (purpose-tagged) plus mirrored probe-
/// position arrays for debug overlays.
///
/// === Multi-batch architecture ===
///
/// Steam Audio supports multiple probe batches attached to a single
/// simulator. Each batch's bake data is keyed by `IPLBakedDataIdentifier`
/// (Reflections / Pathing) and the simulator silently skips batches that
/// don't carry data for the current query type (verified in
/// `vcpkg/.../core/src/core/simulation_manager.cpp` and
/// `baked_reflection_simulator.cpp`).
///
/// We exploit this to split probes by purpose:
///
///   * Reflections batch — dense UNIFORMFLOOR grid (typically 1500-3000
///     probes on a Thief mission). Density matters for IR sampling at any
///     listener position; portal-axis + elevation + emitter passes add
///     coverage in the corners of the visibility graph that the floor grid
///     misses.
///
///   * Pathing batch — sparse ROOM_PORTAL graph (typically 100-300 probes
///     on a Thief mission). One probe at each room centroid, two probes
///     ±offset across each portal plane. Steam Audio's `findAlternatePaths`
///     cost is ~quadratic in probe count when door OBBs invalidate baked
///     paths at runtime — sparse-graph pathing reduces a multi-second hang
///     to microseconds.
///
/// Batch attachment is ASYMMETRIC (PR #6): the REFLECTION simulator gets
/// ONLY batches carrying reflections bake data — Steam Audio v4.7.0's
/// baked-reflections lookup has NO bounds check and indexes across ALL
/// attached batches (baked_reflection_data.cpp), so attaching a data-less
/// pathing batch reads out of bounds whenever pathing probes outnumber
/// reflection probes (crashed MISS2: 949 > 626) and silently blends
/// unrelated probes' reverb otherwise (MISS6: 630 < 833). The PATHING
/// simulator gets both batches, but only consumes the one named on each
/// source's `IPLSimulationInputs::pathingProbes` field (caller must point
/// at `getPathingProbeBatch()`). Do NOT "simplify" back to attach-all.
///
/// Threading: main-thread only. `bakeProbes` is blocking (~10-60s) and
/// reports via an atomic float so a UI thread can poll. Probe batches are
/// read by the reflection-sim worker; callers MUST invoke the supplied
/// `waitForReflectionThread` callback before any batch mutation.

#include "DarknessMath.h"
#include "SteamAudioPathing.h"  // kPathingVisSamplesShip (ProbeBakeParams default)

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

// Forward declarations for Steam Audio opaque handles
struct _IPLContext_t;
typedef _IPLContext_t* IPLContext;
struct _IPLScene_t;
typedef _IPLScene_t* IPLScene;
struct _IPLSimulator_t;
typedef _IPLSimulator_t* IPLSimulator;
struct _IPLProbeBatch_t;
typedef _IPLProbeBatch_t* IPLProbeBatch;

namespace Darkness {

/// Which simulator type this batch is built to serve. The disk-format
/// uint32 values are wire-stable — do not renumber.
enum class ProbePurpose : uint32_t {
    Reflections = 0,
    Pathing     = 1,
};

inline const char *probePurposeString(ProbePurpose p) {
    switch (p) {
        case ProbePurpose::Reflections: return "reflections";
        case ProbePurpose::Pathing:     return "pathing";
    }
    return "unknown";
}

/// Outcome of bake-time filter. Nudge = recoverable displacement;
/// preserves probes just too close to a wall while still dropping those
/// inside solid geometry.
enum class ProbeFilterResult {
    Accept,
    Reject,
    Nudge,
};

struct ProbeFilterDecision {
    ProbeFilterResult result = ProbeFilterResult::Accept;
    /// Unit vector pointing away from the offending surface (Nudge only).
    Vector3 nudgeDir{0.0f, 0.0f, 0.0f};
    /// Engine feet — minimum displacement to satisfy the filter next iter.
    /// Callers add a small margin to avoid boundary ping-pong from jitter.
    float nudgeDistFt = 0.0f;
};

using ProbeFilterFn = std::function<ProbeFilterDecision(const Vector3 &)>;

/// Why a pathing-graph node candidate was created. Drives policy in the
/// post-dedup adaptive-radius pass and in the proximity dedup pass
/// (DoorPair candidates are exempt from dedup so a door is always
/// flanked by two non-overlapping probes that the runtime door
/// validation can reject paths through).
enum class PathingProbePurpose {
    Portal,     ///< Non-door architectural portal — single probe at the centroid.
    DoorPair,   ///< One of a pair of probes flanking a door OBB (±portal normal).
    Centroid,   ///< Room geometric centroid (or sky-snapped floor+5ft fallback).
    Emitter,    ///< Mirror of an ambient-emitter anchor.
    PortalPair, ///< One of a pair of bend probes flanking a NON-door portal
                ///< (±portal normal — Tier 1 "bends" density). Gives the
                ///< path solver an explicit bend point on each side of the
                ///< opening so sound turning a corner takes an audible
                ///< detour around the aperture instead of a straight-through
                ///< approximation. Same geometry as DoorPair but no door OBB
                ///< between the pair.
};

/// Pathing probe layout density tier — the `audio.pathing_probes.density`
/// config knob. Selects how many graph nodes prepareProbeBakeParams emits
/// per architectural feature. Recorded in the .probes v4 header
/// (ProbeFile.h bakedPathingDensity) so a cache baked at one density is
/// never silently consumed by a run configured for another — mismatch
/// triggers the same loud automatic pathing-only re-bake as a numSamples
/// mismatch. Underlying type is fixed (uint32_t) so the value round-trips
/// through the on-disk header verbatim and the enum can be forward-declared.
///
///   • Baseline — Tier 0 (selectable — fast dev bakes / low-end): the
///     original Dark Engine room/portal graph's nodes. 1 probe per room
///     centroid (+ upper centroid for genuinely tall rooms) + 1 per
///     non-door portal center + door flanking pairs + emitter mirrors.
///     Wins the medians (pathing p50 66 vs 85 ms; dev bake 17.7 vs
///     28.4 min at ns8) but has the WORSE worst case (below).
///   • Bends    — Tier 1 (default — user decision 2026-07-12, fidelity
///     first; supersedes the 2026-07-11 baseline flip, whose worst-case
///     numbers were misattributed: at ns8 bends' worst door-spike window
///     measured 316.8 ms vs baseline's 535-696 ms, hidden then by log
///     rate-limiting; see RenderConfig.h audioPathingDensity): baseline,
///     with every NON-door portal's single center probe replaced by a
///     flanking pair (center ± normal × kPairProbeOffsetFt, purpose
///     PortalPair) — the solver's explicit bend points for sound turning
///     corners at every opening/aperture.
///     Cost: +1 probe per non-door portal over baseline.
///
/// At EVERY density a thin-room coverage repair runs after the dedup and
/// inside-door-AABB passes: any room adjacent to a non-door portal that
/// no surviving candidate resolves to gets one interior repair probe
/// (see AudioService::prepareProbeBakeParams "Thin-room coverage
/// repair" — rooms thinner than the flank offset lose their organic
/// candidates at both tiers).
///   • ("high" is reserved for a future Tier 2 — room-span subdivision for
///     long halls. Named in config docs but REJECTED at parse until the
///     tier exists; see RenderConfig.h.)
enum class PathingProbeDensity : uint32_t {
    Unknown  = 0, ///< No pathing batch / pre-v4 data. Never a valid config.
    Baseline = 1,
    Bends    = 2,
};

/// Human-readable density name for logs / headers.
inline const char *pathingProbeDensityName(PathingProbeDensity d) {
    switch (d) {
    case PathingProbeDensity::Unknown:  return "unknown";
    case PathingProbeDensity::Baseline: return "baseline";
    case PathingProbeDensity::Bends:    return "bends";
    }
    return "invalid";
}

/// Inverse of pathingProbeDensityName — the ONLY config-string → enum
/// mapping (single source of truth; the YAML/CLI parsers validate against
/// the same names but store strings). Returns Unknown for anything
/// unrecognized — including the reserved "high" — and callers MUST treat
/// Unknown as a loud config error, never silently default it: a silent
/// default here once mapped every typo to Bends.
inline PathingProbeDensity pathingProbeDensityFromName(const std::string &s) {
    if (s == "baseline") return PathingProbeDensity::Baseline;
    if (s == "bends")    return PathingProbeDensity::Bends;
    return PathingProbeDensity::Unknown;
}

/// One pathing-graph node candidate, supplied by AudioService. Each Room
/// contributes one centroid candidate; each non-door portal contributes
/// one (at the centroid); each door portal contributes two (flanking
/// the door OBB along the portal normal); each persistent ambient
/// emitter contributes one (mirror anchor). ProbeManager runs them
/// through the supplied filter the same way it does floor probes.
struct PathingProbeCandidate {
    Vector3 position{0.0f, 0.0f, 0.0f};
    float   radiusFt = 5.0f;   ///< Influence radius — sized adaptively post-dedup.
    PathingProbePurpose purpose = PathingProbePurpose::Portal;
};

/// Output of `ProbeManager::computeBakePlan` — the exact probe layout a
/// real bake will commit to Steam Audio, EXCEPT no IPL handles are
/// allocated and no IRs / pathing data are baked. Used by the
/// `darknessHeadless probe_plan` verb (Capability A in
/// PLAN.PROBE_DEBUG_TOOLING) to predict per-purpose probe counts
/// without paying the multi-minute bake cost.
///
/// Same code path as a live bake — bakeReflectionBatch and
/// bakePathingBatch both build their candidate lists via
/// `computeBakePlan`. Counts here will match the next live bake
/// byte-for-byte modulo any door-OBB classification that depends on
/// runtime state (in headless mode, door OBBs may be absent — see
/// AudioService::computeProbePlan for the [PROBE_PLAN] WARN banner).
struct ProbeBakePlan {
    /// Engine-feet positions of every reflection-batch probe that would
    /// be committed to the IPL batch (post all placement passes + global
    /// dedup). One entry per probe — `reflectionPositions.size()` is the
    /// total reflection probe count that the next bake will produce.
    std::vector<Vector3> reflectionPositions;
    /// Pathing-batch candidates that would be committed to the IPL
    /// batch (post placement + dedup, post adaptive-radius pass — every
    /// candidate carries its final radiusFt).
    std::vector<PathingProbeCandidate> pathingKept;

    /// Per-pass reflection-batch counters (placement order: floor →
    /// elevation → emitter → global dedup). `globalDeduped` is the
    /// number of probes the final dedup pass dropped.
    int floorKept     = 0;
    int elevationKept = 0;
    int emitterKept   = 0;
    int globalDeduped = 0;

    /// Per-purpose pathing-batch counts (post-dedup, pre-adaptive-radius).
    /// Sum across this map equals `pathingKept.size()`.
    std::map<PathingProbePurpose, int> pathingPerPurpose;

    /// Pathing-batch dedup breakdown — how many candidates the dedup
    /// pass dropped, split by purpose. `dedupDroppedTotal` is the
    /// inclusive total.
    int dedupDroppedTotal      = 0;
    int dedupDroppedCentroids  = 0;
    int dedupDroppedDoorPairs  = 0;
    int dedupDroppedPortalPairs = 0;
    int dedupDroppedOther      = 0;
};

/// Per-bake parameters supplied by AudioService (read from its tuning state).
/// All distance fields are in engine feet (ProbeManager converts to meters
/// at the IPL boundary internally).
struct ProbeBakeParams {
    /// Scene AABB used to position the probe-generation OBB. Engine feet.
    Vector3 sceneMin{0.0f, 0.0f, 0.0f};
    Vector3 sceneMax{0.0f, 0.0f, 0.0f};

    /// Reverb max distance (used as pathing visibility/path range).
    float   propagationMaxDist  = 200.0f;

    /// Reflection bake quality — separate from realtime params so the bake
    /// can afford higher quality without affecting per-frame sim cost.
    int     bakeNumRays          = 4096;
    int     bakeNumBounces       = 8;
    float   bakeDuration         = 4.0f;
    int     bakeDiffuseSamples   = 256;
    int     simulatorThreads     = 0;       ///< 0 = auto
    int     ambisonicsOrder      = 1;       ///< Bake-side ambisonic order
    std::string sceneType        = "default";  ///< "default" or "embree"

    /// Override grid spacing/height in engine feet. Use -1.0f to fall back
    /// to ProbeManager's configured spacing/height.  `spacingFtOverride`
    /// controls UNIFORMFLOOR grid density (smaller = denser fill of big
    /// open cells that FLOOR_POLY single-centroids); `heightFtOverride`
    /// controls how far above the floor each probe sits.
    float   spacingFtOverride    = -1.0f;
    float   heightFtOverride     = -1.0f;

    /// Floor-poly probe candidates (engine ft, Z-up).  One per
    /// upward-facing BSP cell polygon, captured at acoustic-scene build
    /// time.  ProbeManager adds `heightFtOverride`/configured height and
    /// runs each through `probeFilter` (min-wall-clearance + room
    /// membership); survivors become the reflection batch's floor pass.
    std::vector<Vector3> floorProbeCandidates;

    /// Extra elevations (feet, above floor) for replicated probes. Empty
    /// = floor-grid only. Without elevated probes an elevated emitter
    /// routes to the floor probe at its (x,y), misrepresenting geometry.
    std::vector<float> additionalElevations;

    /// Axial probe offset across a portal plane (feet). Used by the
    /// PATHING batch — each surviving RoomService portal contributes two
    /// pathing probes at `center ± normal * portalAxialOffsetFt`.
    /// 1 ft ≈ 0.3 m, small enough that a closed-door OBB fits between
    /// the pair without overlap.
    float portalAxialOffsetFt = 1.0f;

    /// Dedup radius (feet) for the emitter-anchored pass: a candidate is
    /// dropped if any earlier-pass probe is within this distance. Named
    /// "portal" for historical reasons (was shared with the old portal-
    /// ring pass before that was removed).
    float portalDedupRadiusFt = 5.0f;

    /// Elevation-tier sparsity multiplier. Floor probes are binned with
    /// binSize = floorSpacing × this; one elevation probe per bin centroid
    /// per tier. 2.0 = 1:4 ratio (default); 1.0 = 1:1 (legacy).
    float elevationSparsityMul = 2.0f;

    /// Global dedup radius (feet) applied AFTER all placement passes.
    /// Earlier passes have priority (floor always wins). 0 = disabled.
    /// With `probeRoomLookup` set, dedup is room-aware: probes only
    /// collapse against other probes in the SAME room. Without it the
    /// pass falls back to room-agnostic distance dedup, which can
    /// silently strip entire rooms when adjacent rooms have floor-poly
    /// centroids closer than this radius across a thin wall.
    float globalDedupRadiusFt = 2.0f;

    /// Room-membership lookup for the global dedup pass.  Returns the
    /// `RoomService` integer room ID for a probe position, or -1 if the
    /// point is in solid / BSP void. When non-null, the dedup pass only
    /// collapses probes that share a room ID — preserving per-room
    /// coverage across thin walls / adjacent rooms.  When null, dedup
    /// is room-agnostic (legacy behaviour, suitable for UNIFORMFLOOR-
    /// style grid placement where collisions were always real overlap).
    std::function<int(const Vector3 &)> probeRoomLookup;

    /// Pre-bake validity filter applied to every REFLECTIONS-batch candidate.
    /// Returns Accept / Reject / Nudge. Pre-filtering avoids near-zero IRs
    /// from in-wall probes and comb-filtering from near-wall probes. Empty
    /// = keep everything.
    ProbeFilterFn probeFilter;

    /// Per-emitter anchor positions (engine feet, REFLECTIONS batch). One
    /// probe is attempted at each so persistent ambient sources have a
    /// graph node within pathing visibility radius even when the floor/
    /// elevation/portal grid leaves a coverage gap (e.g. wall-mounted
    /// fixtures whose P$Position falls inside the visual wall mesh).
    std::vector<Vector3> emitterPositions;

    /// Filter applied specifically to the emitter-anchored pass. Should
    /// be more permissive than `probeFilter` — typically converts the
    /// grid filter's "in-solid → Reject (no recovery)" into
    /// "in-solid → Nudge toward the nearest open audio room". Empty =
    /// fall back to `probeFilter`.
    ProbeFilterFn emitterProbeFilter;

    /// Informational — what wall clearance (feet) the caller's filter
    /// enforces; logged for traceability. Does NOT derive the filter.
    float minWallClearanceFt = 0.0f;

    // ── Pathing batch (sparse, ROOM_PORTAL-derived) ──────────────────────

    /// True to bake a second probe batch tagged Pathing. When false, no
    /// pathing batch is produced and pathing queries against this mission
    /// return null routing (synthetic-bypass branch in AudioService).
    bool   bakePathingBatch = true;

    /// Caller-supplied pathing graph node candidates. One per room
    /// centroid + two per portal (±offset). Order does not matter;
    /// ProbeManager runs them through `pathingProbeFilter` and dedups
    /// internally. Influence radius travels per-candidate so room-scale
    /// and portal-scale probes can co-exist without false overlap.
    std::vector<PathingProbeCandidate> pathingCandidates;

    /// Filter for the pathing batch. Typically permissive (accept anything
    /// inside a real Room) — pathing probes inside a wall mesh are useless
    /// but a slightly-near-wall pathing probe is fine because we don't
    /// bake reflections off it. Empty = keep everything that landed inside
    /// the candidate list.
    ProbeFilterFn pathingProbeFilter;

    /// Bake-time single-edge distance cap (engine feet) for the pathing
    /// visibility graph (`IPLPathBakeParams::visRange`). Derived by
    /// AudioService::prepareProbeBakeParams from the mission's ROOM_DB
    /// (max intra-room portal-to-portal span / room diameter × margin,
    /// clamped) — ProbeManager deliberately does not depend on
    /// RoomService, so the derivation travels here. <= 0 means the
    /// caller could not derive a cap; bakePathingBatch falls back LOUDLY
    /// to the whole-level range (the pre-cap behaviour). Does NOT bound
    /// multi-hop route length — that is pathRange, which stays
    /// whole-level (see the visRange/pathRange comment in
    /// bakePathingBatch).
    float pathingVisRangeFt = -1.0f;

    /// Pathing visibility sampling count (`IPLPathBakeParams::numSamples`).
    /// MUST be the active profile constant from SteamAudioPathing.h
    /// (kPathingVisSamplesShip / kPathingVisSamplesDev) — AudioService
    /// fills it via activePathingVisSamples() so bake and runtime read
    /// the same selection. Recorded in the .probes header.
    int pathingNumSamples = kPathingVisSamplesShip;

    /// Probe layout density tier the pathing candidates were emitted at
    /// (the `audio.pathing_probes.density` knob). ProbeManager does not
    /// consume it for placement — emission happens in AudioService's
    /// prepareProbeBakeParams — it travels here solely so the bake can
    /// record it into the .probes v4 header for the loader's
    /// density-mismatch check (same policy as pathingNumSamples: loud +
    /// automatic pathing-only re-bake). Default mirrors the config
    /// default (bends — user decision 2026-07-12, fidelity first);
    /// AudioService always overwrites it from the active config.
    PathingProbeDensity pathingDensity = PathingProbeDensity::Bends;

    /// True = pathing-only re-bake: skip the (expensive) reflection IR
    /// bake and carry the CURRENTLY LOADED reflection batch forward into
    /// the output file unchanged. Requires a loaded .probes file whose
    /// reflection batch carries IR data; when that precondition fails the
    /// bake falls back LOUDLY to a full reflection re-bake. Set by
    /// AudioService when a forced pathing re-bake is requested
    /// (--force-pathing-bake, or the automatic v3 header numSamples-
    /// mismatch re-bake).
    bool reuseLoadedReflectionsBatch = false;
};

/// One edge in the pathing-probe visibility graph (Capability C —
/// PLAN.PROBE_DEBUG_TOOLING.md). Endpoints are indices into
/// `ProbeBatchEntry::positions` for the pathing batch.
///
/// NOT a claim that Steam Audio visited this specific edge for any
/// specific voice. The adjacency is computed by Darkness using the
/// same raycast algorithm Steam Audio's pathing baker uses (visRange,
/// numVisSamples) — same algorithm, same inputs, same edges. Layer-2
/// edge coloring (done in the renderer) is a neighborhood-activity
/// heatmap: "voices near this edge's endpoints are perceiving the
/// listener with this EQ quality." It does NOT enumerate Steam Audio's
/// actual visited-probe set per voice — the public C API does not
/// expose that data.
struct PathingEdge {
    int a = -1;
    int b = -1;
};

/// Output of buildPathingAdjacency. Cached on ProbeManager; rebuilt on
/// load/bake. `built` distinguishes "no batch / nothing to test" from
/// "built but empty graph".
struct PathingAdjacency {
    std::vector<PathingEdge> edges;
    /// True once a build pass has completed (success or empty graph).
    /// False on init, after release, or if the build aborted partway.
    bool   built       = false;
    /// Parameters used to build the cached graph. Stored so callers can
    /// surface them in diagnostics and so a rebuild can detect drift.
    float  visRangeFt   = 0.0f;
    int    numVisSamples = 0;
    /// Wall-clock build time (ms) — surfaced in [ADJACENCY_BUILD].
    double buildMs      = 0.0;
};

/// Construction-time wiring.
struct ProbeManagerDeps {
    IPLContext context = nullptr;
    /// Blocks until the reflection-sim worker is idle. Required — called
    /// before any probe-batch register/unregister/release so Steam Audio
    /// never sees a torn batch.
    std::function<void()> waitForReflectionThread;
    /// Blocks until the pathing-sim worker is idle. Optional but
    /// recommended — without it, runtime probe-batch swaps can race
    /// `iplSimulatorRunPathing`. Symmetric with waitForReflectionThread.
    std::function<void()> waitForPathingThread;
};

/// One probe batch held by ProbeManager. Mirrored positions are used by
/// debug overlays and by AudioService's nearest-probe lookups; the IPL
/// handle is the canonical batch.
struct ProbeBatchEntry {
    ProbePurpose          purpose = ProbePurpose::Reflections;
    IPLProbeBatch         iplBatch = nullptr;
    int                   probeCount = 0;
    bool                  hasReflectionsData = false;
    bool                  hasPathingData = false;
    /// Pathing bake profile actually used (pathing entries only; 0 on
    /// reflections entries). Filled by bakePathingBatch with the
    /// EFFECTIVE values (post any fallback), then recorded into the
    /// .probes header by bakeProbes.
    float                 bakedVisRangeFt = 0.0f;
    int                   bakedNumSamples = 0;
    PathingProbeDensity   bakedDensity = PathingProbeDensity::Unknown;
    /// Engine-space positions in placement order (sidecar mirror).
    std::vector<Vector3>  positions;
    /// Per-probe influence radii in engine feet (sidecar mirror). Parallel
    /// to `positions`. Currently populated only for the pathing batch
    /// (where the adaptive Voronoi-ish sizing varies per probe); for the
    /// reflection batch this is left empty (every probe uses the uniform
    /// `spacing` radius). Consumed by the pathing-probe debug overlay
    /// (DarknessRender::renderPathingProbeOverlay) to draw influence
    /// spheres.
    std::vector<float>    radiiFt;
};

/// Owns one or more Steam Audio probe batches + mirrored probe-position
/// arrays + bake-time grid config. Persistent across missions —
/// loadProbes/bakeProbes both call releaseBatches internally before
/// installing new ones.
class ProbeManager {
public:
    explicit ProbeManager(ProbeManagerDeps deps);
    ~ProbeManager();

    ProbeManager(const ProbeManager &) = delete;
    ProbeManager &operator=(const ProbeManager &) = delete;

    /// Bake probes for the current scene. Generates a reflection batch
    /// (UNIFORMFLOOR grid + elevation/portal/emitter passes) and an
    /// optional pathing batch (ROOM_PORTAL nodes from
    /// params.pathingCandidates). Writes a v2 `.probes` file with one
    /// section per batch. Blocking (~10-60s); progress reports via the
    /// atomic float.
    bool bakeProbes(IPLScene scene,
                    const std::string &outputPath,
                    const ProbeBakeParams &params,
                    std::atomic<float> *progress);

    /// Dry-run the Darkness-side probe placement passes WITHOUT touching
    /// Steam Audio's bake (no IRs / no pathing graph). Runs the exact
    /// same code as `bakeReflectionBatch` (lines that produce the
    /// positions vector) and the candidate-filter portion of
    /// `bakePathingBatch`, stopping just before
    /// `iplReflectionsBakerBake` / `iplPathBakerBake`. Used by the
    /// `darknessHeadless probe_plan` verb (Capability A).
    ///
    /// `scene` must be a fully-built IPLScene — the floor-grid
    /// generator raycasts down to find floor positions, so a real
    /// (or stub) scene is required.
    ///
    /// Pre-AudioService-side pathing dedup must already have been
    /// applied to `params.pathingCandidates` (`AudioService::computeProbePlan`
    /// handles this and records the dedup counters separately).
    ///
    /// Returns false if the IPL probe array could not be generated.
    bool computeBakePlan(IPLScene scene,
                         const ProbeBakeParams &params,
                         ProbeBakePlan &out);

    /// Load probes from disk + register batches ASYMMETRICALLY: the
    /// reflection simulator receives only reflections-data batches (the
    /// old "silently ignores" assumption was DISPROVEN — see the
    /// class-level attachment comment: unbounded lookup, PR #6); the
    /// pathing sim receives both and consumes the batch named per-source
    /// via IPLSimulationInputs::pathingProbes.
    bool loadProbes(const std::string &probePath,
                    IPLSimulator reflectionSimulator,
                    IPLSimulator pathingSimulator);

    /// Release every loaded batch. Unregisters from both simulators if
    /// supplied. Safe when no batch is loaded.
    void releaseBatches(IPLSimulator reflectionSimulator,
                        IPLSimulator pathingSimulator);

    /// Standard mission probe path:
    /// ~/darkness/{gameName}/baked_probes/{missionName}.probes
    static std::string getProbeFilePath(const std::string &misPath,
                                         const std::string &gameName = "thief2");

    // ── Read-only accessors ────────────────────────────────────────────

    /// Sum across all batches.
    int getProbeCount() const { return mTotalProbeCount; }
    /// Reflection batch carries baked reflection IRs? False on legacy
    /// or pathing-only loads.
    bool hasReflections() const { return mReflectionsBatch
                                          && mReflectionsBatch->hasReflectionsData; }
    /// Pathing batch present + baked?
    bool hasPathing() const { return mPathingBatch
                                     && mPathingBatch->hasPathingData; }

    /// Pathing bake profile of the CURRENT in-memory data — from the
    /// .probes v3 header on load, or from the just-used bake params after
    /// a bake. 0 = unknown / no pathing batch. AudioService uses the
    /// visRange as the runtime per-voice `inputs.visRange` clamp (edges
    /// longer than the baked cap don't exist in the graph) and compares
    /// the sample count against the active profile constant to trigger
    /// the automatic mismatch re-bake.
    float getBakedPathingVisRangeFt() const { return mBakedPathingVisRangeFt; }
    int   getBakedPathingNumSamples() const { return mBakedPathingNumSamples; }
    /// Probe layout density tier the pathing section was baked at
    /// (.probes v4 header). Unknown = no pathing batch. AudioService
    /// compares this against the active `audio.pathing_probes.density`
    /// config and triggers the loud automatic pathing-only re-bake on
    /// mismatch — a baseline-density cache consumed at bends density
    /// would silently lack the per-portal bend pairs (and vice versa).
    PathingProbeDensity getBakedPathingDensity() const {
        return mBakedPathingDensity;
    }

    /// Reflection bake profile of the CURRENT in-memory data (.probes v3
    /// header fields bakedReflectionRays / bakedProbeDedupRadiusFt).
    /// 0 = unknown. AudioService compares these against the active
    /// profile on every load and emits an UNCONDITIONAL loud [FALLBACK]
    /// on mismatch — no auto-re-bake (the reflection bake is the
    /// expensive half), but dev-tier reverb can never be consumed
    /// silently by a ship run. Pathing-only re-bakes preserve these
    /// values (the header describes the carried-forward blob, not the
    /// active profile).
    int   getBakedReflectionRays() const { return mBakedReflectionRays; }
    float getBakedProbeDedupRadiusFt() const { return mBakedProbeDedupRadiusFt; }

    /// The single batch carrying reflection IRs (or nullptr if none).
    /// Callers use this for `iplProbeBatchGetReverb` / per-probe lookups.
    IPLProbeBatch getReflectionProbeBatch() const {
        return mReflectionsBatch ? mReflectionsBatch->iplBatch : nullptr;
    }
    /// The single batch carrying baked paths (or nullptr if none).
    /// Callers set `inputs.pathingProbes = getPathingProbeBatch()` on
    /// every pathing source per-frame.
    IPLProbeBatch getPathingProbeBatch() const {
        return mPathingBatch ? mPathingBatch->iplBatch : nullptr;
    }

    /// Legacy single-batch accessor — returns the reflection batch (which
    /// also held pathing data in v1 files). Retained for AudioService call
    /// sites that have not yet been split between the two batches.
    IPLProbeBatch getProbeBatch() const { return getReflectionProbeBatch(); }

    /// Reflection-batch probe positions (feet). Rebuilt on bake/load.
    /// Used for the debug overlay and per-source nearest-probe heuristics.
    const std::vector<Vector3> &getProbePositions() const {
        static const std::vector<Vector3> kEmpty;
        return mReflectionsBatch ? mReflectionsBatch->positions : kEmpty;
    }
    /// Pathing-batch probe positions (feet). Empty if no pathing batch
    /// is loaded.
    const std::vector<Vector3> &getPathingProbePositions() const {
        static const std::vector<Vector3> kEmpty;
        return mPathingBatch ? mPathingBatch->positions : kEmpty;
    }
    /// Pathing-batch per-probe influence radii (feet), parallel to
    /// `getPathingProbePositions()`. Empty if no pathing batch is loaded
    /// or if the bake/load path could not recover per-probe radii
    /// (sidecar missing). Consumed by the pathing-probe debug overlay.
    const std::vector<float> &getPathingProbeRadii() const {
        static const std::vector<float> kEmpty;
        return mPathingBatch ? mPathingBatch->radiiFt : kEmpty;
    }

    // ── Pathing adjacency (Capability C debug viz) ────────────────────
    //
    // Static visibility graph over the pathing-batch probes, computed by
    // a Darkness-side replication of Steam Audio's bake-time visibility
    // test (`IPLPathBakeParams.visRange` + `numSamples`). Used ONLY by
    // the show_pathing_graph debug overlay.
    //
    // Honesty note (also above PathingEdge): this is NOT a query against
    // Steam Audio's serialized graph (the public C API doesn't expose
    // that). It's the same algorithm with the same inputs, so the edges
    // SHOULD agree modulo raycast-precision noise.

    /// Raycaster signature used by buildPathingAdjacency. Returns true
    /// if the segment `from`→`to` hits the acoustic scene. AudioService
    /// supplies the engine's mRaycaster bound against the acoustic mesh.
    using PathingAdjacencyRaycaster =
        std::function<bool(const Vector3 &from, const Vector3 &to)>;

    /// Build (or rebuild) the pathing adjacency. O(N²) probe-pair pass
    /// at level-load. `visRangeFt` is the max probe-to-probe distance
    /// to consider (matches Steam Audio's `IPLPathBakeParams.visRange`
    /// guard `areProbesTooFar`); `numVisSamples` is the per-pair ray
    /// count (Steam Audio scatters numSamples² rays in a visRadius
    /// sphere — we currently cast one center-to-center ray as a faithful
    /// reproduction with reduced sample count, sufficient for the
    /// neighborhood-activity heatmap purpose; if visRange or numSamples
    /// drift, callers should rebuild). Emits [ADJACENCY_BUILD].
    /// Safe to call repeatedly; subsequent calls overwrite the cache.
    /// `raycast` must be non-null — without an acoustic-scene raycaster
    /// every pair would be treated as visible and the graph would be
    /// useless; we emit a [VIZ_FALLBACK] line and leave `built=false`.
    void buildPathingAdjacency(float visRangeFt,
                                int   numVisSamples,
                                const PathingAdjacencyRaycaster &raycast);

    /// Clear the cached adjacency. Called from releaseBatches so a
    /// stale graph doesn't survive a level swap.
    void clearPathingAdjacency() { mPathingAdjacency = {}; }

    /// Read-only access. `built` flag tells the caller whether to draw
    /// edges or surface the "NO PATHING BATCH" HUD message.
    const PathingAdjacency &getPathingAdjacency() const {
        return mPathingAdjacency;
    }

    // Bake-time grid config — takes effect on next bake. Does NOT relocate.
    void  setProbeSpacingFt(float ft) { mProbeSpacingFt = std::max(1.0f, std::min(ft, 20.0f)); }
    float getProbeSpacingFt() const { return mProbeSpacingFt; }
    void  setProbeHeightFt(float ft) { mProbeHeightFt = std::max(0.5f, std::min(ft, 20.0f)); }
    float getProbeHeightFt() const { return mProbeHeightFt; }

    // ── T2.4 IR cache stats ──
    //
    // ProbeManager itself owns disk→memory IR data via the IPLProbeBatch
    // handle, but per-voice IR lookups happen in AudioService when it
    // calls iplProbeBatchGetReverb (or when Steam Audio's reflection sim
    // copies a baked IR into a voice's reflectionParams). We accept the
    // hit/miss signal from AudioService so the [PERF refl_cache] line
    // surfaces the IR-population success rate.
    //
    //   recordCacheHit  — a voice acquired baked IR data (irSize>0) from
    //                     the loaded probe batch.
    //   recordCacheMiss — a voice was eligible for reflection but the
    //                     probe lookup produced no usable IR.
    //
    // Lock-free atomics — multiple AudioService loop-step paths can bump.
    void recordCacheHit()  { mCacheHits.fetch_add(1, std::memory_order_relaxed); }
    void recordCacheMiss() { mCacheMisses.fetch_add(1, std::memory_order_relaxed); }

    /// Emit [PERF refl_cache] once per ~1 s window. Caller passes the
    /// active-slot snapshot from the convolution worker pool (we don't
    /// own that; keeping the cross-class wiring explicit) plus the
    /// pool's evictions counter (read out of
    /// `ConvolutionWorker::slotEvictionsTotal` — callers can pass 0 if
    /// they don't have it; the line still reports hits/misses).
    void pollPerfPeriodic(int activeSlots, int maxSlots,
                          uint64_t evictionsSinceLast);

private:
    /// Bake one reflection batch from a UNIFORMFLOOR + elevation/portal/
    /// emitter placement pass + iplReflectionsBakerBake. Returns a fully-
    /// committed batch with reflection IR data attached, or nullptr on
    /// failure. Caller takes ownership of the returned handle.
    bool bakeReflectionBatch(IPLScene scene,
                             const ProbeBakeParams &params,
                             float spacing,
                             float height,
                             std::atomic<float> *progress,
                             const std::string &outputPath,
                             ProbeBatchEntry &outEntry);

    /// Bake one pathing batch from caller-supplied ROOM_PORTAL candidates
    /// + iplPathBakerBake. Returns a committed batch with pathing data
    /// attached, or nullptr if no candidates survive filtering.
    bool bakePathingBatch(IPLScene scene,
                          const ProbeBakeParams &params,
                          float spacing,
                          std::atomic<float> *progress,
                          ProbeBatchEntry &outEntry);

    /// Shared placement pass for the reflection batch. Runs floor-grid
    /// generation + elevation tier + emitter-anchored pass + global
    /// dedup, in exactly the same order and with exactly the same logs
    /// as the historical inline code in `bakeReflectionBatch`. Fills
    /// `plan.reflectionPositions` and the per-pass counters. Returns
    /// false if the floor pass produced no probes (caller treats this
    /// as a hard error — same as historical behavior).
    ///
    /// Called by both `bakeReflectionBatch` (which then commits to an
    /// IPL batch and bakes IRs) and `computeBakePlan` (dry-run, stops
    /// here).
    bool computeReflectionPlacements(IPLScene scene,
                                     const ProbeBakeParams &params,
                                     float spacing,
                                     float height,
                                     ProbeBakePlan &plan);

    /// Shared filter pass for the pathing batch. Walks
    /// `params.pathingCandidates`, runs each through
    /// `params.pathingProbeFilter`, and emits accepted candidates into
    /// `plan.pathingKept` with per-purpose counters. Same code path as
    /// the historical inline filter at the top of `bakePathingBatch`.
    void computePathingPlacements(const ProbeBakeParams &params,
                                  ProbeBakePlan &plan);

    /// Release a single batch from the simulators it is attached to.
    /// Internal use only — releaseBatches walks the vector.
    void releaseEntry(ProbeBatchEntry &entry,
                      IPLSimulator reflectionSimulator,
                      IPLSimulator pathingSimulator);

    ProbeManagerDeps mDeps;

    /// All batches. Owning. Each entry points at one IPLProbeBatch.
    std::vector<ProbeBatchEntry> mBatches;
    /// Convenience pointers into mBatches — the one entry tagged
    /// Reflections and the one tagged Pathing. Re-resolved on every
    /// bake/load.
    ProbeBatchEntry *mReflectionsBatch = nullptr;
    ProbeBatchEntry *mPathingBatch = nullptr;
    /// Sum of probeCount across batches; convenience for accessors.
    int  mTotalProbeCount = 0;

    /// Bake profile of the current in-memory data (see the public
    /// getBakedPathing* / getBakedReflection* accessors). Set from the
    /// .probes v3 header on load and after a successful bake (a
    /// pathing-only re-bake preserves the loaded reflection values);
    /// zeroed on releaseBatches.
    float mBakedPathingVisRangeFt = 0.0f;
    int   mBakedPathingNumSamples = 0;
    PathingProbeDensity mBakedPathingDensity = PathingProbeDensity::Unknown;
    int   mBakedReflectionRays = 0;
    float mBakedProbeDedupRadiusFt = 0.0f;

    /// Negative override → use these values at bake time.
    float mProbeSpacingFt = 5.0f;
    float mProbeHeightFt  = 5.0f;

    // ── T2.4 IR cache stats (see public recordCacheHit/Miss) ──
    std::atomic<uint64_t> mCacheHits{0};
    std::atomic<uint64_t> mCacheMisses{0};

    /// Cached pathing-probe visibility graph (Capability C). Populated
    /// by buildPathingAdjacency; cleared by releaseBatches. Empty +
    /// built=false on init.
    PathingAdjacency mPathingAdjacency;
};

} // namespace Darkness

#endif // __PROBE_MANAGER_H
