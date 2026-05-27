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
/// paths the bake accepted get rejected at runtime. Matches Steam
/// Audio's Unity default. Lower values (e.g. 0.01) admit single stray
/// rays through mesh seams, producing the "audio passes through walls"
/// pathology.
constexpr float kPathingVisThreshold = 0.1f;

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
/// requires deleting the cached .probes file and re-baking — at
/// numSamples=16 that's 256 rays per probe-pair test, so bake time
/// scales up materially (rays = numSamples² × probe_pairs).
constexpr float kPathingVisRadiusFt = 5.0f;

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
