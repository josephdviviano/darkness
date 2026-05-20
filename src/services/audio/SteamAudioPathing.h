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
/// Pure helpers for mapping Steam Audio's 3-band path attenuation
/// (`IPLPathingOutputs::eqCoeffs[3]`) into the engine's per-voice DSP
/// state (portalAttenuation + portalBlocking) plus the runtime closed-
/// door blocking layer.
///
/// Kept header-only so they can be unit-tested without standing up a
/// full Steam Audio simulator / acoustic scene — the live integration in
/// AudioService.cpp consumes the same functions.
///
/// Both inputs and outputs are in [0, 1]:
///   eqCoeffs[i] = 1.0 → unobstructed band, 0.0 → fully attenuated band
///   gain        = scalar voice gain folded into portalAttenuation
///   blocking    = scalar in [0,1] folded into portalBlocking, where the
///                 audio callback maps it to LPF cutoff via
///                 `cutoff_hz = openHz * pow(blockedHz/openHz, blocking)`
///
/// See PLAN.PLAYER_AUDIO_STEAM_PATHING.md "eqCoeffs → audio mapping" for
/// the rationale behind the weighting.

#include <algorithm>
#include <cmath>

namespace Darkness {

/// Result of mapping Steam Audio's pathing outputs into the engine's
/// per-voice DSP gain + LPF-blocking values.
struct PathingDspMapping {
    float gain;     ///< Scalar gain in [0,1] for portalAttenuation
    float blocking; ///< Scalar in [0,1] for portalBlocking → LPF cutoff
};

/// Clamp a single eqCoeff band to [0,1], turning NaN/inf into 1.0
/// (treat ill-formed bake outputs as fully audible so the voice stays
/// hearable rather than silently dying).
inline float sanitizeEqCoeff(float v)
{
    if (!std::isfinite(v)) return 1.0f;
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/// Map a raw `eqCoeffs[3]` triple plus an optional door blocking factor
/// (room-pair lookup, 0 fully open / 1 fully closed) to the per-voice
/// portalAttenuation + portalBlocking pair.
///
/// The door factor multiplies onto all three bands as a `(1 - factor)`
/// passthrough so a fully closed door collapses the bands toward 0 (LPF
/// pegged at blockedHz, gain pegged at silence). Open doors are
/// passthrough — `doorBlocking == 0.0f` leaves eqCoeffs untouched.
///
/// `gainScale` is a runtime multiplier on the final scalar gain only
/// (NOT on the eqCoeffs themselves, so it doesn't affect the LPF
/// blocking factor). 1.0 = identity. Use values > 1 to make
/// through-portal sound louder than the bake would imply when the
/// baked eqCoeffs feel under-cooked — physically unrealistic but
/// useful for compensating sparse probe coverage or a too-strict
/// pathing visibility threshold without paying for a re-bake.
inline PathingDspMapping eqCoeffsToDspMapping(float eqLow,
                                              float eqMid,
                                              float eqHigh,
                                              float doorBlocking,
                                              float gainScale = 1.0f)
{
    eqLow  = sanitizeEqCoeff(eqLow);
    eqMid  = sanitizeEqCoeff(eqMid);
    eqHigh = sanitizeEqCoeff(eqHigh);
    if (!std::isfinite(doorBlocking))    doorBlocking = 0.0f;
    if (doorBlocking < 0.0f)             doorBlocking = 0.0f;
    else if (doorBlocking > 1.0f)        doorBlocking = 1.0f;
    if (!std::isfinite(gainScale) || gainScale < 0.0f) gainScale = 1.0f;

    if (doorBlocking > 0.0f) {
        const float passFactor = 1.0f - doorBlocking;
        eqLow  *= passFactor;
        eqMid  *= passFactor;
        eqHigh *= passFactor;
    }

    PathingDspMapping out{};
    out.gain     = (0.25f * eqLow + 0.50f * eqMid + 0.25f * eqHigh) * gainScale;
    out.blocking = 1.0f - eqHigh;
    return out;
}

} // namespace Darkness

#endif // __STEAM_AUDIO_PATHING_H
