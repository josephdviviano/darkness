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

#ifndef __AUDIO_METERING_H
#define __AUDIO_METERING_H

/// @file AudioMetering.h
/// Shared gain-staging meter for the audio pipeline.
///
/// Extracted so both AudioService.cpp (master-bus chain + per-voice Steam
/// Audio meters) and ConvolutionWorkerPool (per-sub-worker reverb meters)
/// can use the same accumulator without duplicating the struct or hauling
/// it through the public AudioService.h surface.

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Darkness {

/// Peak + RMS accumulator for one stage of the audio pipeline. All values
/// are stored linearly; readers convert to dBFS via `peakDb*()`/`rmsDb*()`.
/// L and R are tracked separately so asymmetric loudness is informative
/// (e.g. binaural pan to far right pushes R hot while L stays quiet).
///
/// dB conversion uses a -120 dB floor so silent windows don't print "-inf".
struct StageMeter {
    float    peakL = 0.0f;
    float    peakR = 0.0f;
    double   sumSqL = 0.0;
    double   sumSqR = 0.0;
    uint64_t sampleCount = 0;

    /// Accumulate peak + sum-of-squares for an interleaved stereo block.
    /// frames is the number of stereo frames (so 2*frames samples total).
    void measure(const float* interleaved, std::size_t frames) {
        if (!interleaved) return;
        for (std::size_t i = 0; i < frames; ++i) {
            float sL = interleaved[i * 2];
            float sR = interleaved[i * 2 + 1];
            float aL = std::fabs(sL);
            float aR = std::fabs(sR);
            if (aL > peakL) peakL = aL;
            if (aR > peakR) peakR = aR;
            sumSqL += static_cast<double>(sL) * sL;
            sumSqR += static_cast<double>(sR) * sR;
        }
        sampleCount += frames;
    }

    /// Accumulate peak + sum-of-squares for separate L and R deinterleaved
    /// buffers (the format the per-voice binaural effect produces before
    /// the interleave step).
    void measureDeinterleaved(const float* chL, const float* chR,
                               std::size_t frames) {
        if (!chL || !chR) return;
        for (std::size_t i = 0; i < frames; ++i) {
            float sL = chL[i];
            float sR = chR[i];
            float aL = std::fabs(sL);
            float aR = std::fabs(sR);
            if (aL > peakL) peakL = aL;
            if (aR > peakR) peakR = aR;
            sumSqL += static_cast<double>(sL) * sL;
            sumSqR += static_cast<double>(sR) * sR;
        }
        sampleCount += frames;
    }

    /// Accumulate peak + sum-of-squares for a mono buffer.  Both L and R
    /// fields are kept synchronized so the [GAIN] dump shows identical
    /// L/R values for mono stages (direct effect, reflection input, etc.)
    /// instead of a misleading -120 dB on the right.
    void measureMono(const float* mono, std::size_t samples) {
        if (!mono) return;
        for (std::size_t i = 0; i < samples; ++i) {
            float s = mono[i];
            float a = std::fabs(s);
            if (a > peakL) peakL = a;
            sumSqL += static_cast<double>(s) * s;
        }
        sampleCount += samples;
        peakR = peakL;
        sumSqR = sumSqL;
    }

    /// Merge another meter's values into this one — used to fold per-
    /// sub-worker convolution meters into the mix node's combined view
    /// at log time without coupling those threads through atomics.
    void mergeIn(const StageMeter &other) {
        if (other.peakL > peakL) peakL = other.peakL;
        if (other.peakR > peakR) peakR = other.peakR;
        sumSqL += other.sumSqL;
        sumSqR += other.sumSqR;
        sampleCount += other.sampleCount;
    }

    void reset() {
        peakL = peakR = 0.0f;
        sumSqL = sumSqR = 0.0;
        sampleCount = 0;
    }

    /// Linear amplitude → dBFS with a -120 dB floor (avoids -inf prints).
    static inline float toDb(float x) {
        return (x > 1e-6f) ? 20.0f * std::log10(x) : -120.0f;
    }

    float peakDbL() const { return toDb(peakL); }
    float peakDbR() const { return toDb(peakR); }

    float rmsDbL() const {
        if (sampleCount == 0) return -120.0f;
        return toDb(static_cast<float>(std::sqrt(sumSqL / sampleCount)));
    }
    float rmsDbR() const {
        if (sampleCount == 0) return -120.0f;
        return toDb(static_cast<float>(std::sqrt(sumSqR / sampleCount)));
    }
};

} // namespace Darkness

#endif // __AUDIO_METERING_H
