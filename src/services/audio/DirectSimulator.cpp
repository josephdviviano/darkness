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

#include "DirectSimulator.h"

#include "AudioLog.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>

#if defined(__APPLE__)
#  include <pthread.h>
#  include <sys/qos.h>
#endif

// Steam Audio C API (for iplSimulatorRunDirect, iplSourceAdd, etc.)
#include <phonon.h>

namespace Darkness {

//------------------------------------------------------
DirectSimulator::DirectSimulator() = default;

//------------------------------------------------------
DirectSimulator::~DirectSimulator()
{
    stop();
}

//------------------------------------------------------
bool DirectSimulator::start()
{
    if (mThread.joinable())
        return true;  // already running
    mShutdown.store(false, std::memory_order_relaxed);
    try {
        mThread = std::thread(&DirectSimulator::workerMain, this);
    } catch (const std::exception &e) {
        // Loud failure — the caller (AudioService::bootstrapFinished)
        // emits the [FALLBACK] line and routes every subsequent frame
        // through runSynchronous(). No-silent-fallbacks.
        std::fprintf(stderr,
            "[FALLBACK] DirectSimulator: worker thread creation FAILED "
            "(%s) — direct sim will run synchronously on the main "
            "thread\n", e.what());
        return false;
    }
    return true;
}

//------------------------------------------------------
void DirectSimulator::stop()
{
    if (!mThread.joinable())
        return;
    mShutdown.store(true, std::memory_order_release);
    mCV.notify_one();
    mThread.join();
}

//------------------------------------------------------
void DirectSimulator::waitForCompletion()
{
    // Spin-wait for the direct sim to complete on its background thread.
    // Called infrequently (voice teardown paths, shutdown, scene
    // destruction, door-geometry registration). The iteration is short
    // (single-digit ms), so a yield loop is fine.
    while (mRunning.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}

//------------------------------------------------------
void DirectSimulator::signal()
{
    // Order matters: flip mRunning=true BEFORE notifying the worker so the
    // main thread's next isRunning() read sees the busy state without
    // racing the worker waking up.
    mRunning.store(true, std::memory_order_release);
    // Self-measure the effective signal cadence from successive signal()
    // timestamps. The caller (AudioService::loopStep) signals once per
    // idle frame, so this is effectively the frame period — the budget
    // the [PERF direct] BUDGET_WARN below compares against. First signal
    // seeds mLastSignalNs without producing an interval.
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
bool DirectSimulator::isAddPending(IPLSource src) const
{
    return std::find(mPendingAdds.begin(), mPendingAdds.end(), src)
        != mPendingAdds.end();
}

//------------------------------------------------------
bool DirectSimulator::removeFromPendingAdds(IPLSource src)
{
    auto it = std::find(mPendingAdds.begin(), mPendingAdds.end(), src);
    if (it == mPendingAdds.end()) return false;
    mPendingAdds.erase(it);
    return true;
}

//------------------------------------------------------
void DirectSimulator::flushPendingAdds(const SourceAddedFn &onAdded)
{
    if (mPendingAdds.empty() || !mSimulator) return;
    for (auto &src : mPendingAdds) {
        iplSourceAdd(src, mSimulator);
        if (onAdded) onAdded(src);
    }
    mPendingAdds.clear();
    mSimulatorDirty = true;
}

//------------------------------------------------------
void DirectSimulator::flushPendingRemovals()
{
    if (mPendingRemovals.empty() || !mSimulator) return;
    for (auto &src : mPendingRemovals) {
        iplSourceRemove(src, mSimulator);
        iplSourceRelease(&src);
    }
    mPendingRemovals.clear();
    mSimulatorDirty = true;
}

//------------------------------------------------------
void DirectSimulator::releasePendingAdds()
{
    // Pending adds never made it into the simulator — just release the
    // handles. Used during scene destruction so the source handles don't
    // leak.
    for (auto &src : mPendingAdds)
        iplSourceRelease(&src);
    mPendingAdds.clear();
}

//------------------------------------------------------
void DirectSimulator::commitIfDirty()
{
    if (!mSimulatorDirty || !mSimulator) return;
    iplSimulatorCommit(mSimulator);
    mSimulatorDirty = false;
}

//------------------------------------------------------
void DirectSimulator::runIteration()
{
    // PLAN.AUDIO_PERF PR 1b — wall-clock-time each iplSimulatorRunDirect
    // call. Timed region is the bare solver call: no allocation, no
    // logging. Shared by the worker loop and the synchronous fallback so
    // both paths produce identical bookkeeping.
    const auto t0 = std::chrono::steady_clock::now();
    iplSimulatorRunDirect(mSimulator);
    const auto t1 = std::chrono::steady_clock::now();
    const float ms =
        std::chrono::duration<float, std::milli>(t1 - t0).count();
    mDirectHist.record(static_cast<double>(ms));
    if (mIterationHook) mIterationHook(ms);

    // Slow-iteration alarm. The direct sim's steady state is single-digit
    // ms; one full 60 fps frame period means occlusion params are lagging
    // ≥2 frames behind the listener. Rate-limited (first 32 + every 64th)
    // to keep the log readable.
    if (ms > 16.0f) {
        static std::atomic<int> sDirectSlowCount{0};
        int sc = sDirectSlowCount.fetch_add(1, std::memory_order_relaxed);
        if (sc < 32 || (sc % 64) == 0) {
            std::fprintf(stderr,
                "[DIRECT_SLOW] iplSimulatorRunDirect took %.1f ms "
                "(occurrence #%d)\n", ms, sc + 1);
        }
    }

    // Bump the completed-cycle counter with release ordering AFTER the
    // solver returned — pairs with the acquire load in completedCycles()
    // so a main-thread reader that observes the new count also observes
    // every output write this iteration made.
    mCompletedCycles.fetch_add(1, std::memory_order_release);
}

//------------------------------------------------------
void DirectSimulator::runSynchronous()
{
    // Degraded mode (worker failed to start): run inline on the calling
    // thread. mRunning is never set — every isRunning() gate in
    // AudioService sees idle, which is correct: nothing is concurrent.
    if (!mSimulator) return;
    runIteration();
    maybeDumpPerf();
}

//------------------------------------------------------
void DirectSimulator::maybeDumpPerf()
{
    // [PERF direct] dump cadence: emit at most once per second to keep
    // the log readable. Reset window after each dump so percentiles
    // describe the last ~1 s of solver behaviour. Mirrors the
    // [PERF pathing] emission in PathingSimulator::workerMain.
    const auto now = std::chrono::steady_clock::now();
    if (mLastPerfDump.time_since_epoch().count() == 0) {
        mLastPerfDump = now;
        return;
    }
    const auto sinceDump =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - mLastPerfDump).count();
    if (sinceDump < 1000) return;

    LatencyHistogram::Percentiles p = mDirectHist.snapshotAndReset(true);
    if (p.n > 0) {
        // Capture the effective signal cadence under the mutex so we
        // don't tear the float read on a 32-bit ABI.
        float cadenceMs = 0.0f;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            cadenceMs = mLastSignalIntervalMs;
        }
        std::fprintf(stderr,
            "[PERF direct] p50=%.2fms p95=%.2fms p99=%.2fms "
            "signalMs=%.2f n=%llu\n",
            p.p50, p.p95, p.p99, cadenceMs,
            static_cast<unsigned long long>(p.n));
        // Budget warning: p95 ≥ 80% of the signal interval means the
        // worker is nearly missing the per-frame cadence on most
        // iterations — loopStep will start skipping frames with
        // [DIRECT_LAG]. Only emit once a cadence has been measured
        // (skip the first window where the caller has only signaled
        // once, and the synchronous fallback where signal() never runs).
        if (cadenceMs > 0.0f
            && p.p95 >= 0.8 * static_cast<double>(cadenceMs)) {
            std::fprintf(stderr,
                "[PERF direct] BUDGET_WARN p95=%.2fms "
                ">= 0.8 * signalMs=%.2f\n",
                p.p95, cadenceMs);
        }
    }
    mLastPerfDump = now;
}

//------------------------------------------------------
void DirectSimulator::workerMain()
{
    // Dedicated thread for the direct-path simulation (distance/air
    // analytic terms on voice-level sources + volumetric-occlusion and
    // transmission ray casts on per-slot sources). Steam Audio's
    // simulateDirect is a plain serial loop over sources — measured at
    // ~5.25 ms p50 / 5.96 p99 on the MISS6 stress run after PR 1a, which
    // was essentially the entire audio main-thread loopStep.
    //
    // Lower QoS to UTILITY for the same reason Pathing/ReflectionSimulator
    // do: ensure the audio callback + convolution workers
    // (USER_INTERACTIVE) preempt us cleanly, and let macOS bias us toward
    // E-cores under load. The results feed the NEXT frame's staging, so a
    // couple ms of extra latency here is invisible (params already lag
    // one loopStep by design).
#if defined(__APPLE__)
    {
        int qosSetRc = pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
        qos_class_t qos = QOS_CLASS_UNSPECIFIED;
        int relPri = 0;
        if (pthread_get_qos_class_np(pthread_self(), &qos, &relPri) == 0) {
            AUDIO_LOG("[SIM_QOS] direct-sim qos=%u rel=%d setRc=%d\n",
                      qos, relPri, qosSetRc);
        }
    }
#endif

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
            runIteration();
        }
        mRunning.store(false, std::memory_order_release);

        // [PERF direct] periodic dump. Done AFTER mRunning is released so
        // we don't keep the busy flag up across a logging branch.
        maybeDumpPerf();
    }
}

} // namespace Darkness
