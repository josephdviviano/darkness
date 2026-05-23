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

#ifndef __REFLECTION_SIMULATOR_H
#define __REFLECTION_SIMULATOR_H

/// @file ReflectionSimulator.h
/// Background reflection-sim worker + deferred source mutation queues.
///
/// Extracted from AudioService — owns the Steam Audio reflection IPLSimulator
/// handle and the dedicated background thread that pumps
/// iplSimulatorRunReflections. Source-add/remove operations issued from the
/// main thread are queued into pending vectors while the worker thread is
/// busy, then flushed during the next idle window by callers (loopStep,
/// destroyAcousticScene, haltAll).
///
/// Threading model — PRESERVED from the original AudioService implementation:
///   • The simulator-side `mIplSimulator` is iterated by ONE background thread
///     (`workerMain`).
///   • The main thread signals the worker by setting `mWant=true` under
///     `mMutex` then notifying `mCV`; it is also the only thread that calls
///     `setSimulatorDirty`, queueSourceAdd/Remove, flushPending, commitIfDirty,
///     waitForCompletion, demoteVoice.
///   • The audio thread does NOT touch this class.
///   • `mRunning` flips true under `mMutex` (after the wait predicate
///     unblocks) and false at the end of the iteration with release/acquire
///     ordering; main-thread code uses `isRunning()` to gate mutations.
///   • Pending source-add / source-remove vectors are touched ONLY from the
///     main thread — guarded by `isRunning()==false` rather than a mutex,
///     matching the existing convention.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Steam Audio opaque handle types (mirror the typedefs from AudioService.h
// so consumers of this header don't need to include phonon.h).
struct _IPLSimulator_t;
typedef _IPLSimulator_t* IPLSimulator;
struct _IPLSource_t;
typedef _IPLSource_t* IPLSource;

namespace Darkness {

class VoicePool;
struct ActiveVoice;

/// Owns the reflection IPLSimulator handle, the background worker thread
/// that pumps `iplSimulatorRunReflections`, the deferred source-add/remove
/// queues, the throttle counter and the rate-divisor configuration knobs.
///
/// AudioService constructs this object inside `bootstrapFinished()` (the
/// thread is started immediately after construction so it can wake the
/// instant the first scene build wants a simulation step). The actual
/// IPLSimulator handle is plugged in via `setSimulator()` from
/// `buildAcousticScene()` once Steam Audio has built it.
class ReflectionSimulator {
public:
    /// Optional drain callback for `demoteVoice()` — invoked before the
    /// reflection source is released so the convolution worker pool can
    /// finish any in-flight frame that still holds a pointer into the
    /// voice's reflection state. Wired by AudioService at construction.
    using ConvolutionDrainFn = std::function<void()>;

    ReflectionSimulator();
    ~ReflectionSimulator();

    // Non-copyable, non-movable (owns a thread + atomics).
    ReflectionSimulator(const ReflectionSimulator&) = delete;
    ReflectionSimulator& operator=(const ReflectionSimulator&) = delete;

    /// Spawn the background worker thread. Safe to call once; subsequent
    /// calls are no-ops. The thread immediately enters its CV wait state —
    /// it does no work until `signal()` is called.
    void start();

    /// Signal the worker to shut down and join the thread. Idempotent.
    void stop();

    /// Block until the in-flight iteration (if any) completes. Spin-wait —
    /// the worker thread releases `mRunning` with release ordering at the
    /// end of each iteration. Direct sim runs inline on the main thread so
    /// it is never concurrent with this call.
    void waitForCompletion();

    /// True while `iplSimulatorRunReflections` is iterating.
    bool isRunning() const {
        return mRunning.load(std::memory_order_acquire);
    }

    /// Install / replace the IPL simulator handle. Called from
    /// `buildAcousticScene()`. The handle is NOT owned (the caller manages
    /// `iplSimulatorRelease`).
    void setSimulator(IPLSimulator simulator) { mSimulator = simulator; }

    /// Read the simulator handle (used by loopStep for setInputs/getOutputs,
    /// by createVoiceSource for source registration, etc.).
    IPLSimulator simulator() const { return mSimulator; }

    /// Install the convolution-worker drain hook. Invoked by `demoteVoice`
    /// to ensure no sub-worker holds a stale reflection-effect pointer when
    /// the source is released.
    void setConvolutionDrainHook(ConvolutionDrainFn fn) {
        mDrainHook = std::move(fn);
    }

    // ── Deferred source mutation ──
    //
    // The reflection simulator's source list is iterated by the background
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
    /// distinguish a never-added source from a committed one).
    bool isAddPending(IPLSource src) const;

    /// Erase a source from the pending-add queue. Returns true if the
    /// source was found.
    bool removeFromPendingAdds(IPLSource src);

    /// Apply queued source-adds to the simulator. Must be called only when
    /// `isRunning() == false`.
    ///
    /// `onAdded` (optional) is invoked once per source AFTER it is added to
    /// the simulator. AudioService uses this to capture the current
    /// `completedCycles()` value into each voice's `reflectionSimCycleAtAdd`
    /// — i.e. the cycle-counter value at FLUSH time, not at QUEUE time.
    /// (See `project_audio_pending_source_race`: capturing per-voice state
    /// at queue time and not at flush time has bit us three times — the
    /// queue-time snapshot can race the simulator visibility, leaving
    /// voices wired with state that the simulator hasn't actually
    /// produced yet.)
    using SourceAddedFn = std::function<void(IPLSource)>;
    void flushPendingAdds(const SourceAddedFn &onAdded = {});

    /// Apply queued source-removals + release each source. Must be called
    /// only when `isRunning() == false`.
    void flushPendingRemovals();

    /// During scene destruction, release any pending-add sources (they
    /// were never added to the simulator).
    void releasePendingAdds();

    /// Sets the "commit needed" flag — read+cleared by `commitIfDirty()`.
    /// Tracks whether deferred adds/removals (or createVoiceSource calls
    /// during the previous idle window) have changed the simulator's
    /// source list. Direct simulator has no analogue (it commits inline).
    void setSimulatorDirty() { mSimulatorDirty = true; }

    /// True if there are pending uncommitted mutations.
    bool isSimulatorDirty() const { return mSimulatorDirty; }

    /// Run `iplSimulatorCommit` if there are pending mutations. Must be
    /// called only when `isRunning() == false`.
    void commitIfDirty();

    /// Signal the worker thread to run one reflection-sim iteration.
    /// Sets `mRunning=true` before notifying the CV so that the very next
    /// `isRunning()` call from the main thread sees the busy state without
    /// racing the worker waking up.
    void signal();

    // ── Throttle / rate divisor ──

    /// Reflection-sim throttle: run every Nth wantReflections request
    /// (1..32). Implements the per-frame frame counter that loopStep
    /// previously held.
    void setThrottle(int n) {
        mThrottle = n < 1 ? 1 : (n > 32 ? 32 : n);
    }
    int  getThrottle() const { return mThrottle; }

    /// Per-tick increment-and-test for the frame counter. Returns true
    /// when the throttle has elapsed (caller should run reflections this
    /// tick) and resets the counter.
    bool throttleTickAndConsume();

    /// Reflection rate divisor: 1=full (48kHz), 2=half (24kHz), 4=quarter
    /// (12kHz). Set BEFORE buildAcousticScene(). Determines per-voice FFT
    /// cost downstream.
    void setRateDivisor(int div) {
        mRateDivisor = (div >= 4) ? 4 : (div >= 2) ? 2 : 1;
    }
    int  getRateDivisor() const { return mRateDivisor; }

    // ── Active-reflection-source counter ──

    /// Number of voices currently holding a reflection IPLSource (whether
    /// committed or pending). Atomic so the debug overlay / perf dump can
    /// read without locking. Incremented in createVoiceSource, decremented
    /// in demoteVoice / removeVoiceSource.
    void incrementActiveSources() {
        mActiveSources.fetch_add(1, std::memory_order_relaxed);
    }
    void decrementActiveSources() {
        mActiveSources.fetch_sub(1, std::memory_order_relaxed);
    }
    int  activeSourceCount() const {
        return mActiveSources.load(std::memory_order_relaxed);
    }

    // ── Sticky-slot demote ──

    /// Release the reflection IPLSource for a voice that has finished
    /// playback AND whose reverb tail has rung out (the sticky-slot
    /// allocator's release condition). Called from AudioService::loopStep
    /// when `sourceEnded && tailTimer <= 0`.
    ///
    /// Reuses the same defer-flush logic as removeVoiceSource: if the
    /// worker thread is running, the IPLSource is queued for cleanup at
    /// the next idle frame; otherwise the source is removed and released
    /// inline. Always calls the convolution drain hook (if installed)
    /// before releasing the source — the worker may hold a staged frame
    /// whose params.ir points at the source's reflection state.
    ///
    /// Sets `voice.reflectionsActive=false`, decrements the active-source
    /// counter, and nulls `voice.reflectionSource` when defer-flush is
    /// required.
    void demoteVoice(ActiveVoice &voice);

private:
    /// Worker thread main — wakes on CV signal, runs one
    /// `iplSimulatorRunReflections`, captures perf counter (gated on
    /// `gAudioLogVerbose`), notifies the reflection mix node, releases
    /// `mRunning`. Loops until `mShutdown` is set.
    void workerMain();

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

    // ── Throttle / rate divisor ──
    int mThrottle = 4;                   ///< run every Nth wantReflections (1..32)
    int mFrameCounter = 0;               ///< increments on each throttleTick
    int mRateDivisor = 2;                ///< 1/2/4 — sample-rate denominator

    // ── Active reflection source counter ──
    std::atomic<int> mActiveSources{0};

    // ── Completed-cycle counter ──
    //
    // Bumped at the END of every worker iteration (after
    // iplSimulatorRunReflections returns) with release ordering. Main
    // thread reads with acquire ordering; the release/acquire pair
    // guarantees that any output writes the simulator made for sources in
    // its current source list are visible to the reader once the
    // post-bump value is observed.
    //
    // Voices capture this counter at source-add time (immediate path) or
    // at flush time (deferred path) into ActiveVoice::reflectionSimCycleAtAdd.
    // The pin gate in loopStep refuses to snapshot the IR until
    //   completedCycles() > voice->reflectionSimCycleAtAdd
    // so we only ever pin AFTER at least one full simulator cycle has run
    // since the source was registered. Without this gate, voices were
    // pinning all-zero IRs (the simulator's "no output yet" state) and
    // sticking with silence for their entire lifetime.
    std::atomic<uint64_t> mCompletedCycles{0};

public:
    uint64_t completedCycles() const {
        return mCompletedCycles.load(std::memory_order_acquire);
    }

private:

    // ── Convolution drain hook ──
    ConvolutionDrainFn mDrainHook;

    // ── Hook called after each iteration to notify the reflection mix
    //     node that simulation has produced fresh output. AudioService
    //     wires this from initReflectionPipeline. Optional: when unset
    //     (no reflection mix node yet) the iteration still runs. ──
public:
    using SimulationRanFn = std::function<void()>;
    void setSimulationRanHook(SimulationRanFn fn) {
        mSimRanHook = std::move(fn);
    }

private:
    SimulationRanFn mSimRanHook;
};

} // namespace Darkness

#endif // __REFLECTION_SIMULATOR_H
