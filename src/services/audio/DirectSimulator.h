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

#ifndef __DIRECT_SIMULATOR_H
#define __DIRECT_SIMULATOR_H

/// @file DirectSimulator.h
/// Background direct-sim worker + deferred source mutation queues.
///
/// Owns the Steam Audio direct-path IPLSimulator handle and a dedicated
/// background thread that pumps iplSimulatorRunDirect. Cloned from
/// PathingSimulator (PLAN.AUDIO_PERF PR 1b) — the direct solver's volumetric
/// occlusion + transmission ray casts are SERIAL per source inside Steam
/// Audio (`sim_threads` does not parallelize simulateDirect), measured at
/// 5.25 ms p50 on the MISS6 stress run after PR 1a. That was ~100% of the
/// audio main-thread loopStep, so it must NOT run on the main loop thread.
///
/// Source-add/remove operations issued from the main thread are queued into
/// pending vectors while the worker thread is busy, then flushed during the
/// next idle window by callers (loopStep, destroyAcousticScene, haltAll).
///
/// Threading model — identical to PathingSimulator / ReflectionSimulator:
///   • The simulator handle (`mSimulator`) is iterated by ONE background
///     thread (`workerMain`).
///   • The main thread signals the worker by setting `mWant=true` under
///     `mMutex` then notifying `mCV`; it is also the only thread that calls
///     queueSourceAdd/Remove, flushPending*, commitIfDirty,
///     waitForCompletion.
///   • The audio thread does NOT touch this class.
///   • `mRunning` flips true inside signal() (BEFORE the CV notify, so the
///     main thread's next `isRunning()` read can never race the worker
///     waking up) and false at the end of the iteration with
///     release/acquire ordering; main-thread code uses `isRunning()` to
///     gate mutations.
///   • Pending source-add / source-remove vectors are touched ONLY from the
///     main thread — guarded by `isRunning()==false` rather than a mutex,
///     matching the existing convention.
///
/// Per-frame staging (`iplSourceSetInputs` and `iplSimulatorSetSharedInputs`)
/// is NOT gated on `isRunning()` — Steam Audio internally double-buffers
/// staging vs active, and both Unity and Unreal reference integrations call
/// these from the game thread while the sim thread is mid-iteration. The
/// loopStep frame shape (harvest → flush → stage → signal, all in the same
/// idle window) means staging in practice only happens while idle anyway.
///
/// Synchronous fallback: if the worker thread fails to start (see start()),
/// AudioService keeps the old synchronous main-thread path alive via
/// runSynchronous() — same bookkeeping (cycle counter, histogram,
/// iteration hook), just executed inline.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "LatencyHistogram.h"

// Steam Audio opaque handle types (mirror the typedefs from AudioService.h
// so consumers of this header don't need to include phonon.h).
struct _IPLSimulator_t;
typedef _IPLSimulator_t* IPLSimulator;
struct _IPLSource_t;
typedef _IPLSource_t* IPLSource;

namespace Darkness {

/// Owns the direct-path IPLSimulator handle, the background worker thread
/// that pumps `iplSimulatorRunDirect`, and the deferred source-add/remove
/// queues.
///
/// AudioService constructs this object at service-construction time (same
/// lifecycle as PathingSimulator); the thread starts in bootstrapFinished
/// and the actual IPLSimulator handle is plugged in via `setSimulator()`
/// from `buildAcousticScene()` once Steam Audio has built it.
class DirectSimulator {
public:
    DirectSimulator();
    ~DirectSimulator();

    // Non-copyable, non-movable (owns a thread + atomics).
    DirectSimulator(const DirectSimulator&) = delete;
    DirectSimulator& operator=(const DirectSimulator&) = delete;

    /// Spawn the background worker thread. Safe to call once; subsequent
    /// calls are no-ops. The thread immediately enters its CV wait state —
    /// it does no work until `signal()` is called.
    ///
    /// Returns false if the thread could not be created (std::system_error
    /// from std::thread) — the caller must then fall back to
    /// runSynchronous() and announce the degraded mode loudly
    /// (no-silent-fallbacks).
    bool start();

    /// True once start() succeeded (worker thread is alive). When false,
    /// callers must use runSynchronous() instead of signal() — signaling a
    /// never-started worker would latch mRunning=true forever and deadlock
    /// every subsequent idle-gated frame.
    bool started() const { return mThread.joinable(); }

    /// Signal the worker to shut down and join the thread. Idempotent.
    void stop();

    /// Block until the in-flight iteration (if any) completes. Spin-wait —
    /// the worker thread releases `mRunning` with release ordering at the
    /// end of each iteration.
    void waitForCompletion();

    /// True while `iplSimulatorRunDirect` is iterating.
    bool isRunning() const {
        return mRunning.load(std::memory_order_acquire);
    }

    /// Install / replace the IPL simulator handle. Called from
    /// `buildAcousticScene()`. The handle is NOT owned (the caller manages
    /// `iplSimulatorRelease`).
    void setSimulator(IPLSimulator simulator) { mSimulator = simulator; }

    /// Read the simulator handle.
    IPLSimulator simulator() const { return mSimulator; }

    /// Per-iteration observation hook, invoked with the wall-clock
    /// duration (ms) of each iplSimulatorRunDirect call — from the worker
    /// thread (or the main thread in runSynchronous fallback mode).
    /// AudioService uses this to keep feeding the pre-existing
    /// `sPerfDirectSimMs` histogram + `sDirectSimPeakMs` scalar so the
    /// `direct_sim` JSONL stage stays A/B-comparable across PR 1b. The
    /// callee must be thread-safe (atomics only).
    using IterationFn = std::function<void(float)>;
    void setIterationHook(IterationFn fn) { mIterationHook = std::move(fn); }

    // ── Deferred source mutation ──
    //
    // The direct simulator's source list is iterated by the background
    // worker thread, so source-add/remove can only run while the worker is
    // idle. Callers use isRunning() to gate the choice between immediate
    // and deferred paths.

    /// Push an IPLSource onto the pending-add queue. The caller is
    /// responsible for invoking `flushPendingAdds()` (with simulator-idle
    /// guaranteed) before relying on the source being in the simulator.
    void queueSourceAdd(IPLSource src) { mPendingAdds.push_back(src); }

    /// Push an IPLSource onto the pending-remove queue.
    void queueSourceRemove(IPLSource src) { mPendingRemovals.push_back(src); }

    /// True if a source handle is sitting in the add queue (used to
    /// distinguish a never-added source from a committed one). The harvest
    /// pass in loopStep uses this to skip reading outputs from sources the
    /// solver has never seen ([PENDING_SKIP] convention — see
    /// project_audio_pending_source_race).
    bool isAddPending(IPLSource src) const;

    /// Erase a source from the pending-add queue. Returns true if the
    /// source was found.
    bool removeFromPendingAdds(IPLSource src);

    /// Apply queued source-adds to the simulator. Must be called only when
    /// `isRunning() == false`.
    ///
    /// `onAdded` (optional) is invoked once per source AFTER it is added to
    /// the simulator. AudioService uses this to capture the current
    /// `completedCycles()` value into each voice's `directSimCycleAtAdd` —
    /// i.e. the cycle-counter value at FLUSH time, not at QUEUE time.
    /// (Same rationale as ReflectionSimulator::flushPendingAdds — see
    /// project_audio_pending_source_race.)
    using SourceAddedFn = std::function<void(IPLSource)>;
    void flushPendingAdds(const SourceAddedFn &onAdded = {});

    /// Apply queued source-removals + release each source. Must be called
    /// only when `isRunning() == false`.
    void flushPendingRemovals();

    /// During scene destruction, release any pending-add sources (they
    /// were never added to the simulator).
    void releasePendingAdds();

    /// Sets the "commit needed" flag — read+cleared by `commitIfDirty()`.
    void setSimulatorDirty() { mSimulatorDirty = true; }

    /// True if there are pending uncommitted mutations.
    bool isSimulatorDirty() const { return mSimulatorDirty; }

    /// Run `iplSimulatorCommit` if there are pending mutations. Must be
    /// called only when `isRunning() == false`.
    void commitIfDirty();

    /// Signal the worker thread to run one direct-sim iteration. Sets
    /// `mRunning=true` before notifying the CV so the main thread's next
    /// `isRunning()` call sees the busy state without racing the worker
    /// waking up. Must only be called while `isRunning() == false` —
    /// signaling a busy worker would queue a re-entry that races the main
    /// thread's next source-mutation flush (the same class of crash as
    /// project_steam_audio_simulator_signal_gate). The loopStep frame
    /// shape guarantees this: signal() is the last step of the idle-only
    /// branch.
    void signal();

    /// Synchronous fallback: run one iplSimulatorRunDirect on the CALLING
    /// thread with full bookkeeping (cycle counter, histogram, iteration
    /// hook). Used when the worker thread failed to start — keeps the
    /// pre-PR-1b behaviour selectable so the loud [FALLBACK] mode is
    /// functional, not just less broken. Must not be mixed with signal()
    /// (callers pick one path via started()).
    void runSynchronous();

    // ── Completed-cycle counter ──
    //
    // Bumped at the END of every iteration (worker or synchronous) with
    // release ordering. Voices capture this counter at source-add time
    // (immediate path) or flush time (deferred path) into
    // ActiveVoice::directSimCycleAtAdd. The harvest gate in loopStep
    // refuses to overwrite a voice's dspNode directParams until
    //   completedCycles() > voice->directSimCycleAtAdd
    // i.e. until at least one full RunDirect has completed since the
    // source entered the simulator. Without this gate, a freshly added
    // source's outputs read back the SimulationData create-time seeds
    // (neutral 1.0 — full volume), audibly popping distant spawns for one
    // frame. Same pattern as ReflectionSimulator::completedCycles.
    uint64_t completedCycles() const {
        return mCompletedCycles.load(std::memory_order_acquire);
    }

private:
    /// Worker thread main — wakes on CV signal, runs one
    /// `iplSimulatorRunDirect` via runIteration(), releases `mRunning`.
    /// Loops until `mShutdown` is set.
    void workerMain();

    /// One timed iplSimulatorRunDirect + all bookkeeping (histogram,
    /// [DIRECT_SLOW], cycle counter, iteration hook). Shared by the worker
    /// loop and the synchronous fallback so both paths stay identical.
    void runIteration();

    /// ~1 Hz [PERF direct] histogram dump + BUDGET_WARN vs the measured
    /// signal cadence. Mirrors PathingSimulator's [PERF pathing] emission.
    void maybeDumpPerf();

    // ── Simulator handle (NOT owned — caller manages release) ──
    IPLSimulator mSimulator = nullptr;

    // ── Background thread state ──
    std::thread mThread;
    std::mutex mMutex;
    std::condition_variable mCV;
    bool mWant = false;                  ///< protected by mMutex
    std::atomic<bool> mRunning{false};   ///< sim iteration in-flight
    std::atomic<bool> mShutdown{false};  ///< stop the worker loop

    // ── Deferred source mutation queues (main-thread-only) ──
    std::vector<IPLSource> mPendingAdds;
    std::vector<IPLSource> mPendingRemovals;

    // ── Simulator commit state ──
    bool mSimulatorDirty = false;

    // ── Completed-cycle counter (see completedCycles()) ──
    std::atomic<uint64_t> mCompletedCycles{0};

    // ── Per-iteration observation hook (see setIterationHook()) ──
    IterationFn mIterationHook;

    // ── [PERF direct] histogram + signal-cadence tracking ──
    //
    // mDirectHist receives one record() per iteration with the wall-clock
    // duration of iplSimulatorRunDirect. mLastSignalNs holds the
    // steady_clock timestamp of the previous signal() so the worker can
    // estimate the effective cadence (one signal per idle loopStep ≈ the
    // frame rate) without piping a config value across.
    // mLastSignalIntervalMs is the most recent interval; updated under
    // mMutex inside signal().
    LatencyHistogram mDirectHist;
    int64_t mLastSignalNs = 0;
    float   mLastSignalIntervalMs = 0.0f;
    std::chrono::steady_clock::time_point mLastPerfDump{};
};

} // namespace Darkness

#endif // __DIRECT_SIMULATOR_H
