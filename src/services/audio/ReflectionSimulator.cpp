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

#include "ReflectionSimulator.h"

#include "AudioLog.h"
#include "LatencyHistogram.h"
#include "VoicePool.h"

#include <algorithm>
#include <chrono>

// Steam Audio C API (for iplSimulatorRunReflections, iplSourceAdd, etc.)
#include <phonon.h>

namespace Darkness {

// ── Perf counters shared with AudioService ──
//
// These atomics are produced by the reflection worker thread and consumed
// by AudioService::dumpAudioStatusPeriodic. Defined here next to their
// only writer; AudioService reads them via extern declarations to avoid
// pulling this header (and ActiveVoice + IPL types) into AudioService.cpp's
// per-frame dump path.
//
// Gating: the verbose-log flag (gAudioLogVerbose) governs whether the
// per-iteration timing is captured. The actual increment-on-completion of
// sReflFramesRun is also gated so the counter only ticks when profiling
// is enabled — preserving the pre-extraction behaviour where the dump
// reset still drains the residual at toggle-off.
std::atomic<float> sReflSimPeakMs{0.0f};
std::atomic<int>   sReflFramesRun{0};
// Per-iteration histogram for iplSimulatorRunReflections. Same writer
// (sim worker thread) and same reader (AudioService dump) pattern as the
// peak/frame counters above.
LatencyHistogram   sPerfReflSimMs;

//------------------------------------------------------
ReflectionSimulator::ReflectionSimulator() = default;

//------------------------------------------------------
ReflectionSimulator::~ReflectionSimulator()
{
    stop();
}

//------------------------------------------------------
void ReflectionSimulator::start()
{
    if (mThread.joinable())
        return;  // already running
    mShutdown.store(false, std::memory_order_relaxed);
    mThread = std::thread(&ReflectionSimulator::workerMain, this);
}

//------------------------------------------------------
void ReflectionSimulator::stop()
{
    if (!mThread.joinable())
        return;
    mShutdown.store(true, std::memory_order_release);
    mCV.notify_one();
    mThread.join();
}

//------------------------------------------------------
void ReflectionSimulator::waitForCompletion()
{
    // Spin-wait for the reflection sim to complete on its background
    // thread. Called infrequently (voice removal, shutdown, probe-batch
    // mutation). Direct sim runs inline on the main thread so it is
    // never concurrent here.
    while (mRunning.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}

//------------------------------------------------------
void ReflectionSimulator::signal()
{
    // Order matters: flip mRunning=true BEFORE notifying the worker so
    // the main thread's next isRunning() read sees the busy state without
    // racing the worker waking up. The worker also writes mRunning=true
    // under the same scheme historically (it was set from loopStep before
    // the notify_one); preserved here.
    mRunning.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mWant = true;
    }
    mCV.notify_one();
}

//------------------------------------------------------
bool ReflectionSimulator::throttleTickAndConsume()
{
    if (++mFrameCounter >= mThrottle) {
        mFrameCounter = 0;
        return true;
    }
    return false;
}

//------------------------------------------------------
bool ReflectionSimulator::isAddPending(IPLSource src) const
{
    return std::find(mPendingAdds.begin(), mPendingAdds.end(), src)
        != mPendingAdds.end();
}

//------------------------------------------------------
bool ReflectionSimulator::removeFromPendingAdds(IPLSource src)
{
    auto it = std::find(mPendingAdds.begin(), mPendingAdds.end(), src);
    if (it == mPendingAdds.end()) return false;
    mPendingAdds.erase(it);
    return true;
}

//------------------------------------------------------
void ReflectionSimulator::flushPendingAdds()
{
    if (mPendingAdds.empty() || !mSimulator) return;
    for (auto &src : mPendingAdds)
        iplSourceAdd(src, mSimulator);
    mPendingAdds.clear();
    mSimulatorDirty = true;
}

//------------------------------------------------------
void ReflectionSimulator::flushPendingRemovals()
{
    if (mPendingRemovals.empty() || !mSimulator) return;
    for (auto &src : mPendingRemovals) {
        iplSourceRemove(src, mSimulator);
        iplSourceRelease(&src);
    }
    mPendingRemovals.clear();
    // Caller decides whether to set the dirty flag — historical behaviour
    // didn't bump it on removal-flush, so we don't either.
}

//------------------------------------------------------
void ReflectionSimulator::releasePendingAdds()
{
    // Pending adds never made it into the simulator — just release the
    // handles. Used during scene destruction so the source handles don't
    // leak.
    for (auto &src : mPendingAdds)
        iplSourceRelease(&src);
    mPendingAdds.clear();
}

//------------------------------------------------------
void ReflectionSimulator::commitIfDirty()
{
    if (!mSimulatorDirty || !mSimulator) return;
    iplSimulatorCommit(mSimulator);
    mSimulatorDirty = false;
}

//------------------------------------------------------
void ReflectionSimulator::workerMain()
{
    // Dedicated thread for reflection simulation (ray-traced reverb).
    // Runs every Nth frame (controlled by the main thread's throttle
    // counter via signal()), can take 50-200ms — latency-tolerant because
    // reverb tails change slowly with listener movement.
    // Separated from direct sim so it never blocks occlusion updates.
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

        // Read profiling flag once per reflection sim cycle.
        const bool profOn = ::Darkness::gAudioLogVerbose;
        if (profOn && mSimulator) {
            auto t0 = std::chrono::steady_clock::now();
            iplSimulatorRunReflections(mSimulator);
            auto t1 = std::chrono::steady_clock::now();
            float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            float prev = sReflSimPeakMs.load(std::memory_order_relaxed);
            if (ms > prev) sReflSimPeakMs.store(ms, std::memory_order_relaxed);
            sReflFramesRun.fetch_add(1, std::memory_order_relaxed);
            // Feed the per-stage histogram — same data as the peak above,
            // but distribution-resolved so the [PERF] dump can compute
            // p50/p95/p99. Useful for spotting "rare 300 ms outlier vs
            // steady 50 ms" where the peak alone is ambiguous.
            sPerfReflSimMs.record(static_cast<double>(ms));
        } else if (mSimulator) {
            iplSimulatorRunReflections(mSimulator);
        }
        if (mSimRanHook) mSimRanHook();
        mRunning.store(false, std::memory_order_release);
    }
}

//------------------------------------------------------
// Stage 2.2 — demote-only fallback for the per-voice reflection source.
//
// Every voice starts with a reflection source (eager allocation in
// createVoiceSource) so the baseline path is "voice has reverb." This
// helper is the only way a voice loses its source mid-life. Called from
// AudioService::loopStep's demote pass when a voice has been outside the
// top-N reflection candidate pool for `mDemoteHysteresisCfg` consecutive
// frames — a deliberately conservative threshold so the demote is a true
// fallback, not the common case.
//
// The dance reuses the existing defer-flush logic (mPendingRemovals) so it
// is safe to call while the reflection sim thread is running — the actual
// iplSourceRemove + iplSourceRelease lands at the next idle flush + commit.
//
// Invariants:
//   • mActiveSources reflects the number of voices in the pool with a
//     non-null reflectionSource (including ones pending add).
//   • PlayerEmitted and Ambient voices are never demoted (the caller
//     filters them out before invoking this helper).
//   • demote on a voice without a reflectionSource is a no-op
//     (idempotent — the caller's pass also short-circuits on null).
//------------------------------------------------------
void ReflectionSimulator::demoteVoice(ActiveVoice &voice)
{
    if (!voice.reflectionSource) return;

    // Pending-promotion fast path: the source was created in this same
    // idle window but never actually added to the simulator. Nothing has
    // computed an IR for it yet, so nothing the convolution worker is
    // holding can reference its state — direct release is safe and the
    // worker drain is unnecessary.
    if (removeFromPendingAdds(voice.reflectionSource)) {
        iplSourceRelease(&voice.reflectionSource);
        voice.framesOutOfTopN = 0;
        mActiveSources.fetch_sub(1, std::memory_order_relaxed);
        AUDIO_LOG("[REFL_DEMOTE] h=%d '%s' (cancelled-pending) active=%d\n",
                  voice.handle, voice.schemaName.c_str(),
                  mActiveSources.load(std::memory_order_relaxed));
        return;
    }

    // The voice's reflection source has been processed by the sim at
    // least once, so the convolution worker may hold a staged slot whose
    // params.ir points into this source's reflection state. We must:
    //   1. Block new staging by clearing reflectionsActive — the audio
    //      thread checks it before staging.
    //   2. Drain any already-staged frames so the worker stops touching
    //      the IR pointer before we release the source.
    // After the drain, the convolution effect (dspNode.reflectionEffect)
    // stays alive and continues to apply silence-padded output until the
    // dspNode is destroyed — its internal tail buffers ring out cleanly
    // even though the source-side IR is gone.
    //
    // Same pattern as removeVoiceSource, but does NOT touch effectsReady
    // (the voice is still alive — just losing its reflection source).
    voice.dspNode.reflectionsActive.store(false, std::memory_order_release);
    if (mDrainHook) mDrainHook();

    if (mRunning.load(std::memory_order_acquire)) {
        // Defer-flush: the loopStep flush at the next idle frame runs
        // iplSourceRemove + iplSourceRelease, after the convolution
        // worker has already been drained above. Safe.
        mPendingRemovals.push_back(voice.reflectionSource);
        voice.reflectionSource = nullptr;
        mSimulatorDirty = true;
    } else if (mSimulator) {
        iplSourceRemove(voice.reflectionSource, mSimulator);
        iplSourceRelease(&voice.reflectionSource);
        mSimulatorDirty = true;
    } else {
        iplSourceRelease(&voice.reflectionSource);
    }
    voice.framesOutOfTopN = 0;
    mActiveSources.fetch_sub(1, std::memory_order_relaxed);

    AUDIO_LOG("[REFL_DEMOTE] h=%d '%s' active=%d\n",
              voice.handle, voice.schemaName.c_str(),
              mActiveSources.load(std::memory_order_relaxed));
}

} // namespace Darkness
