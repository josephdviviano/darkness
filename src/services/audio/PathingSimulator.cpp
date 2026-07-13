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

#include "PathingSimulator.h"

#include "AudioLog.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>

#if defined(__APPLE__)
#  include <pthread.h>
#  include <sys/qos.h>
#endif

// Steam Audio C API (for iplSimulatorRunPathing, iplSourceAdd, etc.)
#include <phonon.h>

namespace Darkness {

//------------------------------------------------------
PathingSimulator::PathingSimulator() = default;

//------------------------------------------------------
PathingSimulator::~PathingSimulator()
{
    stop();
}

//------------------------------------------------------
void PathingSimulator::start()
{
    if (mThread.joinable())
        return;  // already running
    mShutdown.store(false, std::memory_order_relaxed);
    mThread = std::thread(&PathingSimulator::workerMain, this);
}

//------------------------------------------------------
void PathingSimulator::stop()
{
    if (!mThread.joinable())
        return;
    mShutdown.store(true, std::memory_order_release);
    mCV.notify_one();
    mThread.join();
}

//------------------------------------------------------
void PathingSimulator::waitForCompletion()
{
    // Spin-wait for the pathing sim to complete on its background thread.
    // Called infrequently (voice removal, shutdown, probe-batch mutation,
    // door-geometry registration).
    while (mRunning.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}

//------------------------------------------------------
void PathingSimulator::signal()
{
    // Order matters: flip mRunning=true BEFORE notifying the worker so the
    // main thread's next isRunning() read sees the busy state without
    // racing the worker waking up.
    mRunning.store(true, std::memory_order_release);
    // Self-measure the effective throttle interval from successive signal()
    // timestamps. The caller (AudioService::loopStep) owns the actual
    // pathing-throttle config, but rather than couple PathingSimulator to
    // that knob we infer the cadence from how often the worker is woken.
    // First signal seeds mLastSignalNs without producing an interval.
    {
        const auto now = std::chrono::steady_clock::now();
        const int64_t nowNs =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();
        std::lock_guard<std::mutex> lock(mMutex);
        if (mLastSignalNs != 0) {
            const int64_t dn = nowNs - mLastSignalNs;
            if (dn > 0) {
                mLastSignalIntervalMs = static_cast<float>(dn) / 1.0e6f;
            }
        }
        mLastSignalNs = nowNs;
        mWant = true;
    }
    mCV.notify_one();
}

//------------------------------------------------------
bool PathingSimulator::isAddPending(IPLSource src) const
{
    return std::find(mPendingAdds.begin(), mPendingAdds.end(), src)
        != mPendingAdds.end();
}

//------------------------------------------------------
bool PathingSimulator::removeFromPendingAdds(IPLSource src)
{
    auto it = std::find(mPendingAdds.begin(), mPendingAdds.end(), src);
    if (it == mPendingAdds.end()) return false;
    mPendingAdds.erase(it);
    return true;
}

//------------------------------------------------------
void PathingSimulator::flushPendingAdds()
{
    if (mPendingAdds.empty() || !mSimulator) return;
    // [PATH_REG] flush-side trace — mirrors the create-side [PATH_REG] logs
    // in AudioService.cpp. Records every source whose iplSourceAdd was
    // deferred (worker was running at create time) and is now actually
    // entering the simulator's source list. A voice whose source appears
    // here AFTER its first [PATH] non-sentinel log would prove the
    // "voice created but solver never saw it" pending-source race.
    //
    // Rate-limited to first 64 + every 64th to bound bursty load-time spam.
    static std::atomic<int> sFlushLogCount{0};
    const int fc = sFlushLogCount.fetch_add(1, std::memory_order_relaxed);
    const bool verbose = (fc < 64) || ((fc % 64) == 0);
    if (verbose) {
        std::fprintf(stderr,
            "[PATH_REG] flush_begin n=%zu (flush #%d)\n",
            mPendingAdds.size(), fc + 1);
    }
    for (auto &src : mPendingAdds) {
        if (verbose) {
            std::fprintf(stderr,
                "[PATH_REG] flush_add src=%p\n", static_cast<void *>(src));
        }
        iplSourceAdd(src, mSimulator);
        // Mirror into tracked-sources so [PATH_RAW] can find this source.
        // Both vectors are main-thread-only with the same `isRunning()==
        // false` guard, so no extra synchronisation is needed.
        mTrackedSources.push_back(src);
    }
    mPendingAdds.clear();
    mSimulatorDirty = true;
}

//------------------------------------------------------
void PathingSimulator::flushPendingRemovals()
{
    if (mPendingRemovals.empty() || !mSimulator) return;
    for (auto &src : mPendingRemovals) {
        iplSourceRemove(src, mSimulator);
        // Drop from tracked sources before releasing the handle so we
        // don't leave a dangling pointer in the diagnostic-sample list.
        auto it = std::find(mTrackedSources.begin(), mTrackedSources.end(), src);
        if (it != mTrackedSources.end()) mTrackedSources.erase(it);
        iplSourceRelease(&src);
    }
    mPendingRemovals.clear();
}

//------------------------------------------------------
void PathingSimulator::releasePendingAdds()
{
    // Pending adds never made it into the simulator — just release the
    // handles. Used during scene destruction so the source handles don't
    // leak.
    for (auto &src : mPendingAdds)
        iplSourceRelease(&src);
    mPendingAdds.clear();
}

//------------------------------------------------------
void PathingSimulator::trackSourceAdded(IPLSource src)
{
    // Direct-add path (caller already invoked iplSourceAdd because the
    // worker was idle). Mirror into the tracked-sources list so
    // diagnostics can find this source. Skip if already present —
    // defensive against double-notify.
    if (std::find(mTrackedSources.begin(), mTrackedSources.end(), src)
        == mTrackedSources.end()) {
        mTrackedSources.push_back(src);
    }
}

//------------------------------------------------------
void PathingSimulator::trackSourceRemoved(IPLSource src)
{
    auto it = std::find(mTrackedSources.begin(), mTrackedSources.end(), src);
    if (it != mTrackedSources.end()) mTrackedSources.erase(it);
}

//------------------------------------------------------
void PathingSimulator::commitIfDirty()
{
    if (!mSimulatorDirty || !mSimulator) return;
    iplSimulatorCommit(mSimulator);
    mSimulatorDirty = false;
}

//------------------------------------------------------
void PathingSimulator::workerMain()
{
    // Dedicated thread for pathing simulation (probe-graph pathfinding with
    // findAlternatePaths). Worst observed iteration was 11+ seconds on
    // MISS6 when dynamic door OBBs invalidate baked edges and the solver
    // explores an exponential alternate-path space — keeping that on the
    // main thread would freeze the render loop.
    //
    // Lower QoS to UTILITY for the same reason ReflectionSimulator does:
    // ensure the audio callback + convolution workers (USER_INTERACTIVE)
    // preempt us cleanly, and let macOS bias us toward E-cores under load.
#if defined(__APPLE__)
    {
        int qosSetRc = pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
        qos_class_t qos = QOS_CLASS_UNSPECIFIED;
        int relPri = 0;
        if (pthread_get_qos_class_np(pthread_self(), &qos, &relPri) == 0) {
            AUDIO_LOG("[SIM_QOS] pathing-sim qos=%u rel=%d setRc=%d\n",
                      qos, relPri, qosSetRc);
        }
    }
#endif

    // [PERF pathing] dump cadence: emit at most once per second to keep
    // the log readable. Reset window after each dump so percentiles
    // describe the last ~1 s of solver behaviour.
    auto lastPerfDump = std::chrono::steady_clock::now();

    while (true) {
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mCV.wait(lock, [this] {
                return mWant
                       || mShutdown.load(std::memory_order_relaxed);
            });
            if (mShutdown.load(std::memory_order_relaxed))
                break;
            mWant = false;
        }

        if (mSimulator) {
            // T2.5 — wall-clock-time each iplSimulatorRunPathing call and
            // record into the latency histogram. Timed region is the bare
            // solver call: no allocation, no logging.
            const auto t0 = std::chrono::steady_clock::now();
            iplSimulatorRunPathing(mSimulator);
            const auto t1 = std::chrono::steady_clock::now();
            const float ms =
                std::chrono::duration<float, std::milli>(t1 - t0).count();
            mPathingHist.record(static_cast<double>(ms));
            // O2a completion bookkeeping for [DOOR_ROUTE_LATENCY]: stamp
            // the iteration-end time FIRST (relaxed), then publish the
            // cycle count with release — a main-thread reader that
            // acquires the new count is guaranteed to see this (or a
            // newer) end timestamp.
            mLastIterEndNs.store(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    t1.time_since_epoch()).count(),
                std::memory_order_relaxed);
            mCompletedCycles.fetch_add(1, std::memory_order_release);
            // Same threshold as the previous synchronous main-thread timing
            // harness. Rate-limited (first 32 + every 64th thereafter) to
            // keep the log readable. Now logged from the worker thread, so
            // a slow iteration no longer correlates 1:1 with a frame hitch.
            if (ms > 15.0f) {
                static std::atomic<int> sPathingSlowCount{0};
                int sc = sPathingSlowCount.fetch_add(1, std::memory_order_relaxed);
                if (sc < 32 || (sc % 64) == 0) {
                    std::fprintf(stderr,
                        "[PATHING_SLOW] iplSimulatorRunPathing took "
                        "%.1f ms (occurrence #%d, worker thread)\n",
                        ms, sc + 1);
                }
            }

            // T0.1 — [PATH_RAW] log. Sample one source per iteration
            // (round-robin across mTrackedSources) and throttle to ~1 Hz.
            // The point of this log is to observe the RAW pathing output
            // BEFORE any consumer in AudioService applies sentinel
            // detection / door-LPF / gain-scale, so we can distinguish
            // "the solver returned a zero" from "AudioService transformed
            // a non-zero into a zero". See memory:
            // feedback_audio_distinguishing_diagnostics.
            //
            // mTrackedSources mirrors the simulator's source set (mutated
            // by the main thread only while the worker is idle, read
            // here while the worker is "running" — safe because the
            // main thread cannot mutate during this read). If the list
            // is empty (e.g. AudioService hasn't notified us yet via
            // trackSourceAdded), the log is silently skipped this iter.
            if (!mTrackedSources.empty()) {
                const auto now = t1;
                const auto sinceLast =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - mLastRawLogTime).count();
                if (sinceLast >= 1000) {
                    if (mRawSampleIdx >= mTrackedSources.size())
                        mRawSampleIdx = 0;
                    IPLSource samp = mTrackedSources[mRawSampleIdx];
                    mRawSampleIdx++;

                    // Read the raw pathing output from the just-completed
                    // iteration. iplSourceGetOutputs is documented safe
                    // from any thread; here we're the only writer
                    // (iplSimulatorRunPathing just returned) so the read
                    // sees this iteration's freshly committed values.
                    IPLSimulationOutputs out{};
                    iplSourceGetOutputs(samp,
                        IPL_SIMULATIONFLAGS_PATHING, &out);

                    // Steam Audio's pathing output is just a 3-band EQ
                    // (low/mid/high band attenuation) plus an Ambisonic
                    // SH direction field. There is no public "reached"
                    // boolean, no path-count, no per-portal blocking
                    // breakdown — the solver collapses all of that into
                    // the eqCoeffs. We synthesise the diagnostic fields
                    // requested in PLAN.AUDIO_PIPELINE_CLEANUP from
                    // what's actually available:
                    //   reached      — 1 if any band > 0 (path solved
                    //                  to a non-zero attenuation),
                    //                  else 0.
                    //   eqL/M/H      — raw 3-band attenuation (these
                    //                  are the only knobs the solver
                    //                  exposes to engine DSP).
                    //   totalBlock   — 1 - mean(eqCoeffs), an aggregate
                    //                  blocking estimate.
                    //   directBlock  — 1 - eqCoeffs[mid], the band
                    //                  AudioService uses for the
                    //                  scalar gain weighting.
                    //   portalBlock  — 1 - eqCoeffs[high], matches the
                    //                  blocking output of
                    //                  eqCoeffsToDspMapping().
                    //   sentinel     — true when all three bands are
                    //                  bit-identical to the 0.1f
                    //                  pathing-untouched-by-solver
                    //                  sentinel from Steam Audio (see
                    //                  memory:
                    //                  project_steam_audio_gotchas).
                    const float eqL = out.pathing.eqCoeffs[0];
                    const float eqM = out.pathing.eqCoeffs[1];
                    const float eqH = out.pathing.eqCoeffs[2];
                    const float meanEq = (eqL + eqM + eqH) / 3.0f;
                    const bool reached = (eqL > 0.0f || eqM > 0.0f || eqH > 0.0f);
                    const float totalBlock  = 1.0f - meanEq;
                    const float directBlock = 1.0f - eqM;
                    const float portalBlock = 1.0f - eqH;
                    uint32_t b0, b1, b2;
                    std::memcpy(&b0, &eqL, 4);
                    std::memcpy(&b1, &eqM, 4);
                    std::memcpy(&b2, &eqH, 4);
                    constexpr uint32_t kSentinel = 0x3DCCCCCD; // bits of 0.1f
                    const bool sentinel =
                        (b0 == kSentinel && b1 == kSentinel
                         && b2 == kSentinel);

                    std::fprintf(stderr,
                        "[PATH_RAW] src=%p reached=%d eqL=%.3f eqM=%.3f "
                        "eqH=%.3f totalBlock=%.3f directBlock=%.3f "
                        "portalBlock=%.3f sentinel=%d trackedN=%zu\n",
                        static_cast<void*>(samp),
                        reached ? 1 : 0,
                        eqL, eqM, eqH,
                        totalBlock, directBlock, portalBlock,
                        sentinel ? 1 : 0,
                        mTrackedSources.size());

                    mLastRawLogTime = now;
                }
            }
        }
        mRunning.store(false, std::memory_order_release);

        // T2.5 — [PERF pathing] periodic dump. Snapshot the histogram
        // every ~1 s and log p50/p95/p99 + the most recent signal
        // interval (which is effectively the caller's throttle setting).
        // Done AFTER mRunning is released so we don't keep the busy flag
        // up across a logging branch.
        {
            const auto now = std::chrono::steady_clock::now();
            const auto sinceDump =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lastPerfDump).count();
            if (sinceDump >= 1000) {
                LatencyHistogram::Percentiles p =
                    mPathingHist.snapshotAndReset(true);
                if (p.n > 0) {
                    // Capture the effective throttle interval under the
                    // mutex so we don't tear the float read on a 32-bit
                    // ABI. Most platforms make this redundant, but the
                    // cost is negligible vs the iplSimulatorRunPathing
                    // we just performed.
                    float throttleMs = 0.0f;
                    {
                        std::lock_guard<std::mutex> lock(mMutex);
                        throttleMs = mLastSignalIntervalMs;
                    }
                    // O2a staging mix for this window: solved = voices
                    // staged with the PATHING flag set (full validation +
                    // alternate-path search), skipped = IN-RANGE eligible
                    // voices whose memo said nothing changed (Steam Audio
                    // serves the cached outputs; out-of-range voices count
                    // in neither bucket — review F6). skipped >> solved on
                    // idle windows is the dirty-gating working as designed.
                    const uint64_t solvedW =
                        mStagedSolved.exchange(0, std::memory_order_relaxed);
                    const uint64_t skippedW =
                        mStagedSkipped.exchange(0, std::memory_order_relaxed);
                    // unreachableCached: staged-SKIPs the unreachable-route
                    // cache produced (suppressed exhaustive re-solves of
                    // known no-route voices). Loud by user directive.
                    const uint64_t unreachW =
                        mUnreachableCached.exchange(0, std::memory_order_relaxed);
                    // O3-lite scoped-invalidation counters (§8): scopedSolves
                    // = solves on scope-valid voices; scopeSkipped = in-range
                    // voices whose global door gen advanced but whose route
                    // set excluded the moved door(s) — the re-solves scoping
                    // suppressed (the win).
                    const uint64_t scopedW =
                        mScopedSolves.exchange(0, std::memory_order_relaxed);
                    const uint64_t scopeSkipW =
                        mScopeSkipped.exchange(0, std::memory_order_relaxed);
                    std::fprintf(stderr,
                        "[PERF pathing] p50=%.2fms p95=%.2fms p99=%.2fms "
                        "max=%.2fms throttleMs=%.2f n=%llu solved=%llu "
                        "skipped=%llu unreachableCached=%llu "
                        "scopedSolves=%llu scopeSkipped=%llu\n",
                        p.p50, p.p95, p.p99, p.maxMs, throttleMs,
                        static_cast<unsigned long long>(p.n),
                        static_cast<unsigned long long>(solvedW),
                        static_cast<unsigned long long>(skippedW),
                        static_cast<unsigned long long>(unreachW),
                        static_cast<unsigned long long>(scopedW),
                        static_cast<unsigned long long>(scopeSkipW));
                    // Budget warning: p95 ≥ 80% of throttle interval
                    // means we're nearly missing the cadence on most
                    // iterations. Only emit if we have a measured
                    // throttle (skip the first window where the caller
                    // has only signaled once).
                    if (throttleMs > 0.0f
                        && p.p95 >= 0.8 * static_cast<double>(throttleMs)) {
                        std::fprintf(stderr,
                            "[PERF pathing] BUDGET_WARN p95=%.2fms "
                            ">= 0.8 * throttleMs=%.2f\n",
                            p.p95, throttleMs);
                    }
                }
                lastPerfDump = now;
            }
        }
    }
}

} // namespace Darkness
