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

#ifndef __WET_BUS_BEAT_DETECTOR_H
#define __WET_BUS_BEAT_DETECTOR_H

/// @file WetBusBeatDetector.h
///
/// Online detector for the 1–5 Hz amplitude-modulation "beating" that the
/// hybrid-reverb migration in PLAN.HYBRID_REVERB.md targets. The audio
/// thread pushes one envelope sample per audio frame (peak or RMS of the
/// summed wet stereo block); the main thread periodically autocorrelates
/// the envelope over lag bins corresponding to 1–5 Hz and reports the
/// peak score.
///
/// Beating signature: a high normalised autocorrelation peak (> ~0.3) in
/// the 0.2–1.0 s lag range. A clean steady-state ambience has very low
/// autocorrelation outside lag=0 — any persistent amplitude rhythm at
/// 1–5 Hz lights this up.
///
/// Threading: single producer (audio thread, `push` is atomic-bump on the
/// write index + a non-atomic float write), single consumer (main-thread
/// periodic dump, `analyze` linearises the ring buffer via the write index
/// snapshot). Torn float reads during a race are diagnostically harmless —
/// they perturb one sample of a 256-sample buffer.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace Darkness {

class WetBusBeatDetector {
public:
    /// 256 samples × ~21.3 ms/frame ≈ 5.5 s history window. Power of two
    /// keeps the modulo cheap. The detector's lag window (0.2–1.0 s) fits
    /// comfortably inside this with room for the autocorrelation tail.
    static constexpr int kEnvCap = 256;

    WetBusBeatDetector() {
        for (auto &v : mBuf) v = 0.0f;
    }

    WetBusBeatDetector(const WetBusBeatDetector &) = delete;
    WetBusBeatDetector &operator=(const WetBusBeatDetector &) = delete;

    /// Audio thread: push one envelope sample per audio frame. `envSample`
    /// should be a non-negative scalar (peak or RMS of the wet stereo block).
    void push(float envSample) {
        if (!std::isfinite(envSample)) envSample = 0.0f;
        if (envSample < 0.0f)          envSample = -envSample;
        const uint64_t w = mWriteIdx.fetch_add(1, std::memory_order_acq_rel);
        mBuf[static_cast<int>(w & (kEnvCap - 1))] = envSample;
    }

    struct AnalysisResult {
        float envMean   = 0.0f;  ///< Mean of the envelope over the window
        float envRMS    = 0.0f;  ///< Std-dev of the envelope (after mean-removal)
        float acPeak    = 0.0f;  ///< Best normalised autocorrelation in [minHz, maxHz]
        float acLagSec  = 0.0f;  ///< Lag at the peak (seconds)
        float acFreqHz  = 0.0f;  ///< 1/acLagSec
        bool  beating   = false; ///< True iff acPeak > threshold
        int   samples   = 0;     ///< How many samples were in the window
    };

    /// Main thread: linearise the ring, compute mean+std-dev, then
    /// autocorrelate over lag bins matching the 1–5 Hz band.
    ///
    /// `frameRateHz` is the rate at which `push` is called (= audio device
    /// rate / device frame size = e.g. 48000/1024 ≈ 46.9 Hz).
    AnalysisResult analyze(float frameRateHz,
                           float minHz    = 1.0f,
                           float maxHz    = 5.0f,
                           float threshold = 0.3f) const {
        AnalysisResult r{};
        const uint64_t w = mWriteIdx.load(std::memory_order_acquire);
        if (w == 0) return r;

        // Linearise. If we have fewer than kEnvCap samples so far, only use
        // what's there; otherwise read the most recent kEnvCap entries in
        // chronological order. Without this step the autocorrelation would
        // see ring-wrap discontinuities.
        const int total = static_cast<int>(std::min<uint64_t>(w, kEnvCap));
        std::array<float, kEnvCap> linear{};
        const uint64_t start = (w >= kEnvCap) ? w - kEnvCap : 0;
        for (int i = 0; i < total; ++i) {
            linear[i] = mBuf[static_cast<int>((start + i) & (kEnvCap - 1))];
        }

        // Mean + (mean-removed) variance.
        double sum = 0.0;
        for (int i = 0; i < total; ++i) sum += linear[i];
        const double mean = sum / total;
        double sumSq = 0.0;
        for (int i = 0; i < total; ++i) {
            const double d = linear[i] - mean;
            sumSq += d * d;
        }
        const double variance = sumSq / total;
        r.envMean = static_cast<float>(mean);
        r.envRMS  = static_cast<float>(std::sqrt(variance));
        r.samples = total;

        // Silence floor — autocorrelating numerical noise just produces
        // spurious peaks. 1e-7 covers the legitimate "wet bus off" case
        // (no reflection voices, peak ≈ 0).
        if (variance < 1e-14) return r;

        // Lag bounds. lag (in samples) = frameRate / freq, so a 1 Hz beat
        // is at lag = frameRate, a 5 Hz beat at lag = frameRate/5.
        int minLag = static_cast<int>(std::floor(frameRateHz / maxHz));
        int maxLag = static_cast<int>(std::ceil(frameRateHz / minHz));
        if (minLag < 1)         minLag = 1;
        if (maxLag > total - 4) maxLag = total - 4;  // keep at least 4 samples in the AC sum
        if (maxLag < minLag)    return r;            // window too short for this band

        double bestPeak = 0.0;
        int    bestLag  = 0;
        for (int lag = minLag; lag <= maxLag; ++lag) {
            double ac = 0.0;
            const int n = total - lag;
            for (int i = 0; i < n; ++i) {
                ac += (linear[i] - mean) * (linear[i + lag] - mean);
            }
            ac /= (n * variance);  // normalise to [-1, 1]
            if (ac > bestPeak) { bestPeak = ac; bestLag = lag; }
        }
        r.acPeak   = static_cast<float>(bestPeak);
        r.acLagSec = (bestLag > 0) ? bestLag / frameRateHz : 0.0f;
        r.acFreqHz = (bestLag > 0) ? frameRateHz / bestLag : 0.0f;
        r.beating  = (r.acPeak > threshold);
        return r;
    }

private:
    std::array<float, kEnvCap> mBuf;
    std::atomic<uint64_t>      mWriteIdx{0};
};

} // namespace Darkness

#endif // __WET_BUS_BEAT_DETECTOR_H
