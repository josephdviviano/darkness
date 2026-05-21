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

#ifndef __PATHING_SIMULATOR_H
#define __PATHING_SIMULATOR_H

/// @file PathingSimulator.h
/// Background pathing-sim worker + deferred source mutation queues.
///
/// Owns the Steam Audio pathing IPLSimulator handle and a dedicated background
/// thread that pumps iplSimulatorRunPathing. Mirrors ReflectionSimulator's
/// pattern — the pathing solver is also CPU-heavy (50–11000 ms on dense baked
/// graphs once dynamic door geometry invalidates baked edges and
/// findAlternatePaths starts an exponential search) so it must NOT run on the
/// main loop thread.
///
/// Source-add/remove operations issued from the main thread are queued into
/// pending vectors while the worker thread is busy, then flushed during the
/// next idle window by callers (loopStep, destroyAcousticScene, haltAll).
///
/// Threading model — identical to ReflectionSimulator:
///   • The simulator handle (`mSimulator`) is iterated by ONE background thread
///     (`workerMain`).
///   • The main thread signals the worker by setting `mWant=true` under
///     `mMutex` then notifying `mCV`; it is also the only thread that calls
///     `setSimulatorDirty`, queueSourceAdd/Remove, flushPending, commitIfDirty,
///     waitForCompletion.
///   • The audio thread does NOT touch this class.
///   • `mRunning` flips true under `mMutex` (after the wait predicate
///     unblocks) and false at the end of the iteration with release/acquire
///     ordering; main-thread code uses `isRunning()` to gate mutations.
///   • Pending source-add / source-remove vectors are touched ONLY from the
///     main thread — guarded by `isRunning()==false` rather than a mutex,
///     matching the existing convention.
///
/// Per-frame staging (`iplSourceSetInputs` and `iplSimulatorSetSharedInputs`)
/// is NOT gated on `isRunning()` — Steam Audio internally double-buffers
/// staging vs active, and both Unity and Unreal reference integrations call
/// these from the game thread while the sim thread is mid-iteration. The
/// throttle interval is the only thing controlling staging cadence.

#include <atomic>
#include <condition_variable>
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

/// Owns the pathing IPLSimulator handle, the background worker thread that
/// pumps `iplSimulatorRunPathing`, and the deferred source-add/remove
/// queues.
///
/// AudioService constructs this object inside `bootstrapFinished()` (the
/// thread is started immediately after construction so it can wake the
/// instant the first scene build wants a simulation step). The actual
/// IPLSimulator handle is plugged in via `setSimulator()` from
/// `buildAcousticScene()` once Steam Audio has built it.
class PathingSimulator {
public:
    PathingSimulator();
    ~PathingSimulator();

    // Non-copyable, non-movable (owns a thread + atomics).
    PathingSimulator(const PathingSimulator&) = delete;
    PathingSimulator& operator=(const PathingSimulator&) = delete;

    /// Spawn the background worker thread. Safe to call once; subsequent
    /// calls are no-ops. The thread immediately enters its CV wait state —
    /// it does no work until `signal()` is called.
    void start();

    /// Signal the worker to shut down and join the thread. Idempotent.
    void stop();

    /// Block until the in-flight iteration (if any) completes. Spin-wait —
    /// the worker thread releases `mRunning` with release ordering at the
    /// end of each iteration.
    void waitForCompletion();

    /// True while `iplSimulatorRunPathing` is iterating.
    bool isRunning() const {
        return mRunning.load(std::memory_order_acquire);
    }

    /// Install / replace the IPL simulator handle. Called from
    /// `buildAcousticScene()`. The handle is NOT owned (the caller manages
    /// `iplSimulatorRelease`).
    void setSimulator(IPLSimulator simulator) { mSimulator = simulator; }

    /// Read the simulator handle.
    IPLSimulator simulator() const { return mSimulator; }

    // ── Deferred source mutation ──
    //
    // The pathing simulator's source list is iterated by the background
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
    void flushPendingAdds();

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

    /// Signal the worker thread to run one pathing-sim iteration. Sets
    /// `mRunning=true` before notifying the CV so the main thread's next
    /// `isRunning()` call sees the busy state without racing the worker
    /// waking up.
    void signal();

private:
    /// Worker thread main — wakes on CV signal, runs one
    /// `iplSimulatorRunPathing`, logs `[PATHING_SLOW]` if elapsed > 15 ms,
    /// releases `mRunning`. Loops until `mShutdown` is set.
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
};

} // namespace Darkness

#endif // __PATHING_SIMULATOR_H
