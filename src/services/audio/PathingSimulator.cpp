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
#include <cstdio>

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
    {
        std::lock_guard<std::mutex> lock(mMutex);
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
    for (auto &src : mPendingAdds)
        iplSourceAdd(src, mSimulator);
    mPendingAdds.clear();
    mSimulatorDirty = true;
}

//------------------------------------------------------
void PathingSimulator::flushPendingRemovals()
{
    if (mPendingRemovals.empty() || !mSimulator) return;
    for (auto &src : mPendingRemovals) {
        iplSourceRemove(src, mSimulator);
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
            auto t0 = std::chrono::steady_clock::now();
            iplSimulatorRunPathing(mSimulator);
            auto t1 = std::chrono::steady_clock::now();
            const float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
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
        }
        mRunning.store(false, std::memory_order_release);
    }
}

} // namespace Darkness
