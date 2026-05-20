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

#ifndef __LATENCY_HISTOGRAM_H
#define __LATENCY_HISTOGRAM_H

/// @file LatencyHistogram.h
///
/// Lightweight log-bucket latency histogram for the audio pipeline. The
/// existing scalar peak counters (`sReflSimPeakMs`, `peakMs`, etc.) only
/// catch the worst single iteration since the last dump — useful for
/// "did anything spike" but blind to the steady-state distribution. The
/// histogram resolves that by tracking the full latency distribution and
/// computing p50/p95/p99 at log time.
///
/// Layout: log-spaced bins, base 1.3, anchored at 10 µs. With 50 bins
/// that covers ~10 µs to ~13 s — plenty for any audio-pipeline stage
/// short of "the worker is hung." The 1.3 base (vs the earlier 1.5)
/// gives ~30% per-bin resolution, fine enough to distinguish e.g. a
/// 100 ms sim cycle from a 130 ms one (different bins). The earlier
/// base-1.5 collapsed most of the 100-500 ms range into 3 bins
/// (137 / 206 / 309 ms centers), making sim-cycle distribution
/// unreadable. Bin counts are atomic so multiple writers (e.g. several
/// sub-worker threads sharing one histogram) are safe; the reader
/// serialises at the call site (main-thread periodic dump). Snapshot
/// uses atomic exchange-to-zero per-bucket so each dump window is
/// independent. A racy producer write between exchanges can drift the
/// percentile by at most one bin — acceptable for diagnostics.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace Darkness {

class LatencyHistogram {
public:
    static constexpr int    kNumBins = 50;
    static constexpr double kBase    = 1.3;
    static constexpr double kAnchorMs = 0.01;  // 10 µs

    LatencyHistogram() {
        for (auto &b : mBuckets) b.store(0, std::memory_order_relaxed);
    }

    LatencyHistogram(const LatencyHistogram &) = delete;
    LatencyHistogram &operator=(const LatencyHistogram &) = delete;

    /// Record one observation. Producer-side, hot path.
    /// NaN/negative are ignored (defensive — should not happen).
    void record(double ms) {
        if (!std::isfinite(ms) || ms < 0.0) return;
        int idx = binFor(ms);
        mBuckets[idx].fetch_add(1, std::memory_order_relaxed);
    }

    struct Percentiles {
        double   p50  = 0.0;
        double   p95  = 0.0;
        double   p99  = 0.0;
        uint64_t n    = 0;
    };

    /// Snapshot and reset (atomic exchange-to-zero per bucket). Reader-side.
    /// `reset=false` returns the same percentiles but leaves counts in place.
    Percentiles snapshotAndReset(bool reset = true) {
        std::array<uint64_t, kNumBins> snap{};
        uint64_t total = 0;
        for (int i = 0; i < kNumBins; ++i) {
            snap[i] = reset
                ? mBuckets[i].exchange(0, std::memory_order_relaxed)
                : mBuckets[i].load(std::memory_order_relaxed);
            total += snap[i];
        }
        Percentiles r{};
        r.n = total;
        if (total == 0) return r;

        const uint64_t t50 = total / 2;
        const uint64_t t95 = (total * 95) / 100;
        const uint64_t t99 = (total * 99) / 100;
        uint64_t cumul = 0;
        bool got50 = false, got95 = false, got99 = false;
        for (int i = 0; i < kNumBins; ++i) {
            cumul += snap[i];
            if (!got50 && cumul >= t50) { r.p50 = binCenterMs(i); got50 = true; }
            if (!got95 && cumul >= t95) { r.p95 = binCenterMs(i); got95 = true; }
            if (!got99 && cumul >= t99) { r.p99 = binCenterMs(i); got99 = true; }
            if (got99) break;
        }
        return r;
    }

private:
    /// Bin layout:
    ///   bin 0          = ms ≤ 10 µs
    ///   bin i (1..N-2) = (kAnchorMs * kBase^(i-1)) < ms ≤ (kAnchorMs * kBase^i)
    ///   bin N-1        = ms > kAnchorMs * kBase^(N-2)  (overflow)
    static int binFor(double ms) {
        if (ms <= kAnchorMs) return 0;
        const double idx = std::log(ms / kAnchorMs) / std::log(kBase) + 1.0;
        if (idx >= static_cast<double>(kNumBins - 1)) return kNumBins - 1;
        const int i = static_cast<int>(idx);
        if (i < 0) return 0;
        return i;
    }

    /// Geometric center of bin `i`, used as the percentile value.
    static double binCenterMs(int i) {
        if (i == 0) return kAnchorMs * 0.5;       // mid of (0, 10 µs]
        if (i == kNumBins - 1) return kAnchorMs * std::pow(kBase, kNumBins - 2);
        const double lo = kAnchorMs * std::pow(kBase, i - 1);
        const double hi = kAnchorMs * std::pow(kBase, i);
        return std::sqrt(lo * hi);
    }

    std::array<std::atomic<uint64_t>, kNumBins> mBuckets;
};

/// RAII timer. Records elapsed wall-clock ms into the histogram on dtor.
/// Use as: `ScopedLatencyTimer t(myHistogram);` around a code block.
class ScopedLatencyTimer {
public:
    explicit ScopedLatencyTimer(LatencyHistogram &h)
        : mHist(h), mStart(std::chrono::steady_clock::now()) {}

    ~ScopedLatencyTimer() {
        const auto end = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(end - mStart).count();
        mHist.record(ms);
    }

    ScopedLatencyTimer(const ScopedLatencyTimer &) = delete;
    ScopedLatencyTimer &operator=(const ScopedLatencyTimer &) = delete;

private:
    LatencyHistogram &mHist;
    std::chrono::steady_clock::time_point mStart;
};

} // namespace Darkness

#endif // __LATENCY_HISTOGRAM_H
