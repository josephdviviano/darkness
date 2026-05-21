/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
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

#ifndef __DARKNESS_AUDIO_UNITS_H
#define __DARKNESS_AUDIO_UNITS_H

/// @file AudioUnits.h
/// Single source of truth for audio-subsystem unit conversions and
/// shared numerical constants used across the engine↔Steam Audio boundary.

namespace Darkness {

// Engine feet ↔ Steam Audio meters.
constexpr float kFeetToMeters = 0.3048f;
constexpr float kMetersToFeet = 1.0f / kFeetToMeters;

/// Distance epsilon (engine feet) guarding division/normalisation when
/// source and listener are essentially co-located.
constexpr float kDistanceEpsilonFt = 0.001f;

/// Steam Audio irradiance-integration min-distance clamp. Must match
/// between bake (IPLReflectionsBakeParams::irradianceMinDistance) and
/// runtime (IPLSimulationSharedInputs::irradianceMinDistance) or baked
/// IRs are normalised against a different singularity threshold than
/// runtime IRs.
constexpr float kIrradianceMinDistanceMeters = 1.0f;

/// Convolution effect's internal delay line is `irDuration *
/// reflectionSampleRate` samples; we must feed that many zero-samples
/// through before eliding the apply, or the resume after silence smears
/// the stale tail. 2× margin covers the parametric-tail FDN residue.
inline int reflSilentSkipFrames(float realtimeDurationSec,
                                int reflectionSampleRate,
                                int reflectionFrameSize)
{
    if (reflectionFrameSize <= 0 || reflectionSampleRate <= 0) return 1;
    const float irSamples = realtimeDurationSec
                          * static_cast<float>(reflectionSampleRate);
    const float callbacks = irSamples / static_cast<float>(reflectionFrameSize);
    const int withMargin = static_cast<int>(callbacks * 2.0f) + 1;
    return (withMargin < 1) ? 1 : withMargin;
}

// Engine defaults — overwritten at audio init by device negotiation /
// YAML config. Provided here so struct member initialisers have sane
// values before the device opens.
constexpr unsigned int kDefaultDeviceSampleRate = 48000;
constexpr unsigned int kDefaultDeviceFrameSize = 1024;

} // namespace Darkness

#endif // __DARKNESS_AUDIO_UNITS_H
