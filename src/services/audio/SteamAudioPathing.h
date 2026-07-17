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

#ifndef __STEAM_AUDIO_PATHING_H
#define __STEAM_AUDIO_PATHING_H

/// @file SteamAudioPathing.h
/// Pure helpers mapping Steam Audio's 3-band path attenuation
/// (IPLPathingOutputs::eqCoeffs) into per-voice DSP state
/// (portalAttenuation + portalBlocking) plus runtime door-closed blocking.
/// Header-only so it can be unit-tested without a live Steam Audio scene.
/// All values in [0,1]: eqCoeffs[i] = 1 unobstructed, 0 fully blocked.

#include "AudioUnits.h"

#include <algorithm>
#include <cmath>

namespace Darkness {

/// Pathing-solver ray-visibility threshold. Used by BOTH bake
/// (IPLPathBakeParams::threshold) and runtime
/// (IPLSimulationInputs::visThreshold) — they must agree or marginal
/// paths the bake accepted get rejected at runtime. This is the
/// fraction of the numVisSamples² inter-probe rays that must clear the
/// acoustic scene for a probe-pair to count as mutually visible (and
/// thus form a graph edge the path solver can route through).
///
/// Was 0.1 (Steam Audio's Unity default). RAISED to 0.25 on 2026-06-26
/// to HARDEN against cross-wall edges. With the 5 ft visibility sphere
/// (kPathingVisRadiusFt) a probe near a thin wall scatters sample
/// points onto BOTH sides of the wall; a handful of those sample-pairs
/// have unobstructed LOS *around* the wall geometry and, at threshold
/// 0.1, the resulting 10%-visible pair was enough to forge a spurious
/// edge between two acoustically-separate rooms. That edge let the
/// path solver "find" a route through the wall → the reflection send
/// (gated by pathing) painted that source's reverb into the listener's
/// room. Requiring 25% of the numVisSamples² rays to clear rejects these thin
/// "around-the-seam" slivers while keeping genuine doorways (where the
/// large majority of sample-pairs see each other).
///
/// Lower values (e.g. 0.01) admit single stray rays through mesh seams
/// — the classic "audio passes through walls" pathology. Higher values
/// risk dropping legitimate grazing-angle doorway edges (and can
/// reintroduce the wet-bus tremolo the radius/sample bump fought), so
/// this is a tuning knob: validate by playtest, back off toward 0.15 if
/// real doorways go silent.
///
/// CHANGING THIS REQUIRES A RE-BAKE: delete the cached .probes file
/// under ~/darkness/thief2/baked_probes/ so the pathing graph is
/// rebuilt at the new threshold. Bake and runtime read the same
/// constant, so they stay in agreement automatically.
constexpr float kPathingVisThreshold = 0.25f;

/// Sampling-sphere radius (engine feet) used by Steam Audio's pathing
/// visibility test (`path_visibility.cpp:50-91` — to determine
/// probe-to-probe visibility, Steam Audio scatters numSamples points
/// inside a sphere of this radius around each probe and traces
/// numSamples² rays against the acoustic scene). Used by BOTH bake
/// (`IPLPathBakeParams::radius`) and runtime
/// (`IPLSimulationInputs::visRadius`). MUST be identical at bake and
/// runtime; otherwise edges baked at one radius are silently rejected
/// by the runtime visibility check and pathing collapses to the 0.1f
/// untouched-by-solver sentinel.
///
/// IMPORTANT — what this is NOT:
///   • This is NOT the per-probe `influence.radius` (IPLSphere.radius).
///     That is the listener/source containment sphere used by
///     `ProbeBatch::getInfluencingProbes` (probe_tree.cpp:158) to
///     decide which probes a listener/source binds to.
///     `influence.radius` is set per-probe in AudioService.cpp's
///     adaptive-radius pass — they are independent dials.
///   • This radius does NOT determine whether sound can bypass doors.
///     Door-blocking is enforced by the acoustic-mesh ray test
///     (`scene.isOccluded` in path_visibility.cpp:67/73/76): the door
///     OBB lives in the acoustic scene, so any ray cast through the
///     sampling spheres that would cross a closed door hits the OBB
///     and is counted as occluded. Two probes' sampling spheres can
///     overlap a door OBB on opposite sides — every ray between them
///     still goes through the OBB and the visibility edge is rejected.
///
/// Past sessions tried to make this small "so the sphere doesn't
/// bridge across a door OBB". That reasoning was wrong — the OBB
/// blocks rays, not spheres. Larger values give the bake more samples
/// to find a clear line of sight through doorways and around corners,
/// improving graph robustness against mesh seams and small artifacts.
///
/// Was Steam Audio's Unity default `bakingVisibilityRadius = 1.0 m`
/// (~3.28 ft). Raised to 5.0 ft (≈ 1.52 m) on 2026-05-26 as part of
/// the pathing-stabilization experiment alongside numVisSamples
/// 4 → 16: the wet-bus tremolo on long-lived ambients (m06winged*
/// gain swinging 0.0 ↔ 0.6 across cycles) is consistent with
/// stochastic visibility decisions flipping edges valid/invalid
/// between solver cycles. A larger sampling sphere gives the bake
/// more chances to find a clear LOS through doorways and around
/// corners, reducing the borderline-visibility region.
///
/// Must remain identical at bake and runtime. Changing this value
/// requires deleting the cached .probes file and re-baking
/// (rays = numSamples² × probe_pairs, so bake cost scales with the
/// sample count squared).
constexpr float kPathingVisRadiusFt = 5.0f;

/// Pathing visibility sampling count — the single source of truth for
/// BOTH the bake (`IPLPathBakeParams::numSamples`, ProbeManager.cpp
/// bakePathingBatch) and the runtime
/// (`IPLSimulationSettings::numVisSamples`, AudioService.cpp pathing-
/// simulator create). Steam Audio scatters numSamples points in each
/// probe's kPathingVisRadiusFt sphere and traces numSamples² rays per
/// probe pair, so this is a QUADRATIC bake-cost knob (4 → 16 rays/pair;
/// 16 → 256). Bake and runtime MUST match: a runtime value differing from
/// the bake silently re-rejects bake-accepted edges and pathing collapses
/// to the 0.1f untouched-by-solver sentinel.
///
/// Two profiles, selected by the `--bake-quality dev` CLI flag
/// (RenderConfig devBakeProfile → AudioService::setDevBakeProfile —
/// one flag feeds both the bake and the runtime simulator so they can
/// never diverge within a run).
///
/// WHY both are 4 (user decision 2026-07-11, PLAN.PATHING_DESIGN.md §6):
/// 4 is Valve's shipping value (Steam Audio Unity/Unreal plugin default).
/// The 2026-05-26 raise to 16 was fighting stochastic edge flapping /
/// wet-bus tremolo, but the later kPathingVisThreshold 0.1 → 0.25
/// hardening addresses that borderline-visibility flicker directly —
/// at 16 we were paying 16× Valve's ray budget (256 vs 16 rays/pair,
/// quadratic) at bake AND at every runtime findPaths/findAlternatePaths
/// re-validation, for insurance the threshold now provides. Regression
/// guard for edge stability is the [PATH_DIAG_DUMP] eq-flap diagnostics,
/// not a bigger sample count. The Ship/Dev split is retained structurally
/// (profile plumbing, .probes header, mismatch re-bake) in case the
/// values ever need to diverge again.
///
/// CROSS-RUN mismatches (cache baked under one profile, run under the
/// other) are caught by the .probes v3 header, which records the bake's
/// numSamples (ProbeFile.h bakedPathingNumSamples). The loader compares
/// it against the active profile constant and triggers a loud automatic
/// pathing re-bake instead of running with a torn edge set.
constexpr int kPathingVisSamplesShip = 4;
constexpr int kPathingVisSamplesDev  = 4;

/// ── Coverage-based probe-layout / bake-range constants ──────────────────
///
/// Graph-reduction lever (§10 machinery, re-scoped to WR REGIONS by the
/// portal-first overhaul — PLAN.PATHING_DESIGN.md §36-40): air regions
/// whose apertures sit further than kPathingCoverageRadiusFt from any
/// same-region graph anchor receive interior HubFill probes
/// (visibility-aware greedy k-center, Gonzalez farthest-point); the fill
/// also SEEDS zero-anchor regions carrying real aperture demand and
/// STITCHES disconnected anchor components with stepping stones (§40b).
/// So on a typical mission the fill fires in MANY regions (MISS2: ~100
/// seeds + fill; MISS15: 155) — that spread is normal, not over-fire.
/// The bake cap is then derived from the POST-fill coverage:
///
///   visRange = clamp(kPathingCoverageMarginMul × max_region(governing),
///              [kPathingCoverageVisRangeMinFt, kPathingCoverageVisRangeMaxFt])
///
/// where governing(region) = max over the region's aperture demand of the
/// distance to the nearest same-region probe, excluding the aperture's
/// own probe (a route entering via an aperture lands on it and must HOP
/// onward — the hop length is what visRange has to cover; the ×2 margin
/// bounds anchor-to-anchor hops via the demand point between them).
/// kPathingCoverageRadiusFt is 75 ft per the §10 research (k-center /
/// disk-cover decomposition).
///
/// Recorded in the .probes header (bakedPathingRCovFt /
/// bakedPathingCoverageFt) so a cache baked under a different coverage
/// radius (or the pre-coverage max-span derivation — v4 files, which
/// read back 0) is caught by AudioService::pathingBakeCoverageMismatch
/// and triggers the same loud automatic pathing-only re-bake as a
/// numSamples/density mismatch.
constexpr float kPathingCoverageRadiusFt      = 75.0f;
/// Interior (listener/source containment) coverage radius for BIG open
/// regions: floor demand may DRIVE probe placement only when its nearest
/// visible anchor is farther than this. Deliberately 2x the aperture
/// coverage radius: the measured §14 over-fire was floor-chasing at 75 ft
/// in hundreds of room-sized units; at 150 ft an ordinary room can never
/// trigger (any aperture probe is closer), while a 300-400 ft warehouse
/// hall gets the handful of interior probes that stop torch placement
/// from being the de-facto coverage plan (MISS9 field observation,
/// 2026-07-16: open regions got far too few probes and kept emitter
/// anchors were carrying interior coverage).
constexpr float kPathingInteriorCoverageRadiusFt = 150.0f;
constexpr float kPathingCoverageMarginMul     = 2.0f;
constexpr float kPathingCoverageVisRangeMinFt = 100.0f;
constexpr float kPathingCoverageVisRangeMaxFt = 200.0f;
/// Per-REGION ceiling on fill + seed + stitch probes. Purely a runaway
/// guard; hitting it is announced loudly and the region's residual
/// coverage/connectivity then shows up in the [PATHING_BAKE_RANGE]
/// derivation and [REGION_PARITY]. NOTE: tuned when the fill was
/// room-scoped; regions are ~8x larger units (review item — it did not
/// bind on MISS2/6/7/15).
constexpr int kPathingFillMaxPerRegion = 24;

/// ── Pathing probe LAYOUT generation ─────────────────────────────────────
///
/// Bumped whenever the probe LAYOUT algorithm changes in a way none of the
/// other .probes header fields capture (numSamples / density / R_cov all
/// unchanged), so cached bakes from the previous layout trigger the loud
/// automatic pathing-only re-bake instead of loading silently.
///   0 = pre-portal-first (room centroids + portal/bend pairs; v4/v5 files
///       read back 0)
///   1 = portal-first (WR-aperture nodes, region coverage fill —
///       PLAN.PATHING_DESIGN.md §36-38)
constexpr uint32_t kPathingLayoutVersion = 1u;

/// (The bend-pair aperture size threshold that lived here —
/// kPathingApertureBendThresholdFt = 6 ft on the ROOM_DB portal inradius —
/// is REMOVED, and deliberately not re-tunable: the number it thresholded
/// was a ROOM_DB box reading, and ROOM_DB size is fiction. Measured: three
/// physically-adjacent real arches read 6.300 ft there (real openings
/// 1.9-2.8 ft, overstated ~2.5x) and were silenced by a 5% margin, while
/// genuine mid-air box boundaries read 5.00 ft and were kept. The aperture
/// test is now the WR oracle — a portal is real iff world-geometry portal
/// area exists at it — with no size rule at all.
/// PLAN.PATHING_DESIGN.md §34/§36; WorldApertureData.h.)

/// ── Warmup first-solve stagger (PLAN.PATHING_DESIGN.md §15/§16) ─────────
///
/// Max never-solved-yet voices the staging pass may hand to any single
/// signaled `iplSimulatorRunPathing` iteration.
///
/// WHY: Steam Audio's runtime alternate-path search (`findShortestPath`,
/// path_finder.cpp) is an A* with NO range/hop/cost bound — on a FAILING
/// (occluded / unreachable) search it terminates only by DRAINING the
/// whole reachable component, so its cost is edge-count-bound, not
/// distance-bound. §15 measured this directly: Stage A's -11% edges moved
/// the worst iteration 1959 -> 1744 ms, exactly proportional. There is no
/// runtime search budget in the C API to cap an individual drain (capping
/// it needs an SA fork — lever C).
///
/// What IS ours to control is how many drains land in the SAME iteration.
/// At warmup ~10 voices become pathing-eligible together and their FIRST
/// solves all stage into one iteration; several drain, and the iteration
/// costs ~1.7 s. The step-2 unreachable-route cache already stops these
/// from RE-solving (warmup re-solves 56 -> 4), but each voice's first
/// solve still has to happen once. Spreading those first solves over
/// consecutive iterations converts one ~1.7 s stall into a short series
/// of bounded ones — the same total work, none of it in a single spike.
///
/// SAFE because a never-solved voice is ALREADY silent: it holds
/// pathTargetValid=false and is parked by the F3 pending/hold machinery
/// until its first covering solve lands. Deferring its first solve by a
/// few ticks EXTENDS an existing correct state; it cannot create
/// staleness (there is no prior route to go stale). This is precisely why
/// only the never-solved class is deferrable: door-invalidated and
/// movement/room re-solves carry the <=150 ms door-route gate and a
/// stale-but-audible route, so they are NEVER staggered.
///
/// 2 = the starting value (PLAN §16). Lower flattens the warmup further
/// (each iteration can host at most this many drains) at the cost of more
/// ticks before every voice owns a route; raise it if warmup routes are
/// perceptibly late. Voices over the cap keep their pathSolvePending
/// latch and are ordered by aging-then-audibility, so the backlog drains
/// in a bounded number of ticks (see the selection pre-pass in
/// AudioService::loopStep) — [PERF pathing] firstSolveDeferred= /
/// firstSolveBacklog= make it visible.
constexpr uint32_t kPathingFirstSolvesPerIteration = 2;

/// Legacy helper: returns the fixed `kPathingVisRadiusFt × kFeetToMeters`
/// regardless of its `spacingFt` argument. The argument is preserved
/// only to minimize call-site churn; new code should reference
/// `kPathingVisRadiusFt` directly. Both bake and runtime call sites
/// MUST agree (verified inline at each call site — see
/// ProbeManager.cpp::bakePathingBatch and
/// AudioService.cpp::loopStep pathing-source setup).
/// @deprecated use kPathingVisRadiusFt directly.
inline float pathingVisRadiusMeters(float spacingFt)
{
    (void)spacingFt;
    return kPathingVisRadiusFt * kFeetToMeters;
}

struct PathingDspMapping {
    float gain;     ///< scalar gain → portalAttenuation
    float blocking; ///< [0,1] → portalBlocking → LPF cutoff
};

/// Clamp a band to [0,1]; NaN/inf → 1.0 (treat ill-formed bake outputs
/// as audible rather than silently muting).
inline float sanitizeEqCoeff(float v)
{
    if (!std::isfinite(v)) return 1.0f;
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/// Map eqCoeffs[3] + door blocking factor (0 open / 1 closed) to per-voice
/// (gain, blocking). Door multiplies all bands by (1-doorBlocking).
///
/// `gainScale` multiplies the scalar gain only (not eqCoeffs → not the
/// LPF). 1.0 = identity. Use >1 to compensate sparse probe coverage.
/// `blockingScale` multiplies the LPF blocking factor only; 0 = LPF
/// never closes, 1 = legacy (`1 - eqHigh`).
/// `gainBandWeights` controls how the 3 bands sum into gain. Default
/// {0.25, 0.50, 0.25} is mid-heavy (roughly A-weighted). Sums != 1 act
/// as flat boost/cut.
inline PathingDspMapping eqCoeffsToDspMapping(float eqLow,
                                              float eqMid,
                                              float eqHigh,
                                              float doorBlocking,
                                              float gainScale = 1.0f,
                                              float blockingScale = 1.0f,
                                              float gainWeightLow  = 0.25f,
                                              float gainWeightMid  = 0.50f,
                                              float gainWeightHigh = 0.25f)
{
    eqLow  = sanitizeEqCoeff(eqLow);
    eqMid  = sanitizeEqCoeff(eqMid);
    eqHigh = sanitizeEqCoeff(eqHigh);
    if (!std::isfinite(doorBlocking))    doorBlocking = 0.0f;
    if (doorBlocking < 0.0f)             doorBlocking = 0.0f;
    else if (doorBlocking > 1.0f)        doorBlocking = 1.0f;
    if (!std::isfinite(gainScale) || gainScale < 0.0f) gainScale = 1.0f;
    if (!std::isfinite(blockingScale) || blockingScale < 0.0f) blockingScale = 1.0f;
    if (blockingScale > 1.0f) blockingScale = 1.0f;
    // Clamp weights to [0,1]; sum can still be 0..3 (intentional — callers
    // use >1 sums for boost effects).
    if (!std::isfinite(gainWeightLow)  || gainWeightLow  < 0.0f) gainWeightLow  = 0.0f;
    if (!std::isfinite(gainWeightMid)  || gainWeightMid  < 0.0f) gainWeightMid  = 0.0f;
    if (!std::isfinite(gainWeightHigh) || gainWeightHigh < 0.0f) gainWeightHigh = 0.0f;
    if (gainWeightLow  > 1.0f) gainWeightLow  = 1.0f;
    if (gainWeightMid  > 1.0f) gainWeightMid  = 1.0f;
    if (gainWeightHigh > 1.0f) gainWeightHigh = 1.0f;

    if (doorBlocking > 0.0f) {
        const float passFactor = 1.0f - doorBlocking;
        eqLow  *= passFactor;
        eqMid  *= passFactor;
        eqHigh *= passFactor;
    }

    PathingDspMapping out{};
    out.gain     = (gainWeightLow  * eqLow
                  + gainWeightMid  * eqMid
                  + gainWeightHigh * eqHigh) * gainScale;
    out.blocking = (1.0f - eqHigh) * blockingScale;
    return out;
}

} // namespace Darkness

#endif // __STEAM_AUDIO_PATHING_H
