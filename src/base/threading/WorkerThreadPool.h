/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 Darkness contributors
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
 *    along with this program; if not, see <https://www.gnu.org/licenses/>.
 *
 *    Inspired by Godot Engine's WorkerThreadPool (MIT License).
 *    See: https://github.com/godotengine/godot
 *
 *****************************************************************************/

#ifndef __WORKER_THREAD_POOL_H
#define __WORKER_THREAD_POOL_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace Darkness {

// ─── Types ───────────────────────────────────────────────────────────────────

using TaskID = int64_t;
using GroupID = int64_t;

constexpr TaskID INVALID_TASK_ID = -1;
constexpr GroupID INVALID_GROUP_ID = -1;

enum class Priority { High, Low };

// ─── WorkerThreadPool ────────────────────────────────────────────────────────

/// General-purpose thread pool for CPU-bound background work.
///
/// Designed for AI NPC updates (pathfinding, sensing, decisions), lightmap
/// baking, asset processing, and similar latency-tolerant parallel work.
/// Audio keeps its own dedicated threads — this pool is not used for
/// real-time audio processing.
///
/// Two-tier priority: High-priority tasks are always drained before
/// Low-priority tasks. A configurable ratio limits how many threads can
/// work on low-priority tasks simultaneously, preventing starvation of
/// high-priority work that arrives while low-priority tasks are running.
///
/// Usage pattern for AI (per-AI tasks, not phased groups):
///   for (auto& ai : scheduledThisFrame)
///       pool.submit([&ai]{ ai->fullUpdate(); }, Priority::High);
///   // main thread does other work...
///   pool.waitForAll();
///
/// Thread-local scratch buffers (e.g. pathfinding A* workspace) can be
/// indexed via getThreadIndex(), which returns a stable 0..N-1 index
/// for the calling worker thread.
///
/// IMPORTANT: Do not call waitFor() or waitForAll() from inside a pool
/// task — this can deadlock if all worker threads block waiting for tasks
/// that require a free worker to execute.

class WorkerThreadPool {
public:
    /// Construct the pool. Does not start threads — call init() to start.
    WorkerThreadPool();

    /// Destructor. Calls shutdown() if not already shut down.
    ~WorkerThreadPool();

    // Non-copyable, non-movable
    WorkerThreadPool(const WorkerThreadPool &) = delete;
    WorkerThreadPool &operator=(const WorkerThreadPool &) = delete;

    /// Start worker threads.
    /// @param threadCount  Number of worker threads. -1 = auto (audio-aware).
    /// @param reservedThreads  Threads to reserve for other systems (audio, main).
    ///                         Only used when threadCount == -1.
    /// @param lowPriorityRatio  Fraction of threads allowed to run low-priority
    ///                          tasks (0.0–1.0). Default 0.3 = 30%.
    void init(int threadCount = -1, int reservedThreads = 3,
              float lowPriorityRatio = 0.3f);

    /// Graceful shutdown: signals all threads to stop, wakes them, joins them.
    /// Pending tasks are discarded. In-flight tasks run to completion.
    void shutdown();

    /// @return true if init() has been called and shutdown() has not.
    bool isRunning() const;

    /// @return Number of worker threads (0 if not initialized).
    int getThreadCount() const;

    // ─── Single task submission ──────────────────────────────────────────

    /// Submit a single task for execution. Returns immediately.
    /// Returns INVALID_TASK_ID if the pool is shutting down.
    /// @param work      The callable to execute on a worker thread.
    /// @param priority  High or Low priority.
    /// @return A TaskID that can be used with isComplete() / waitFor().
    TaskID submit(std::function<void()> work,
                  Priority priority = Priority::High);

    /// Check whether a task has completed.
    /// Returns true for INVALID_TASK_ID or unknown IDs (defensive).
    bool isComplete(TaskID id) const;

    /// Block the calling thread until the task completes.
    /// No-op for INVALID_TASK_ID or already-completed tasks.
    void waitFor(TaskID id);

    /// Block until ALL submitted tasks (single + group) have completed.
    void waitForAll();

    /// Remove completed tasks and fully-completed groups from internal maps.
    /// Call periodically (e.g., once per frame after waitForAll) to prevent
    /// unbounded memory growth. Only safe when no other thread is calling
    /// isComplete()/waitFor() on the pruned tasks.
    void pruneCompleted();

    // ─── Group task submission ───────────────────────────────────────────

    /// Submit a batch of independent work items that share a callable.
    /// The callable receives an index from 0 to count-1.
    /// @param work      Called once per element with its index.
    /// @param count     Number of work items. 0 = returns immediately-complete group.
    /// @param priority  Priority for all items in the group.
    /// @return A GroupID for tracking completion.
    GroupID submitGroup(std::function<void(uint32_t)> work, uint32_t count,
                        Priority priority = Priority::High);

    /// Check whether all items in a group have completed.
    bool isGroupComplete(GroupID id) const;

    /// Block until all items in the group complete.
    void waitForGroup(GroupID id);

    /// @return Number of completed items in the group (for progress tracking).
    uint32_t getGroupProgress(GroupID id) const;

    // ─── Thread identity ────────────────────────────────────────────────

    /// Returns a stable index (0..threadCount-1) for the calling thread,
    /// or -1 if called from a non-pool thread. Useful for indexing into
    /// thread-local scratch buffers (e.g. pathfinding workspace).
    int getThreadIndex() const;

private:
    // ─── Internal task representation ────────────────────────────────────

    struct Group; // Forward declaration

    struct Task {
        TaskID id = INVALID_TASK_ID;
        std::function<void()> work;
        Priority priority = Priority::High;
        std::atomic<bool> completed{false};

        // Group membership (nullptr for standalone tasks).
        // Direct pointer avoids re-locking mQueueMutex for group progress update.
        Group *group = nullptr;
    };

    struct Group {
        GroupID id = INVALID_GROUP_ID;
        uint32_t totalCount = 0;
        std::atomic<uint32_t> completedCount{0};

        // Shared callable — each task calls this with its index
        std::function<void(uint32_t)> work;
    };

    // ─── Worker thread entry point ───────────────────────────────────────

    void workerMain(int threadIndex);

    /// Try to pop a task from the queues. Returns nullptr if none available.
    /// Caller must hold mQueueMutex.
    Task *popTask(bool allowLowPriority);

    // ─── State ───────────────────────────────────────────────────────────

    std::vector<std::thread> mThreads;
    std::atomic<bool> mShutdown{false};
    std::atomic<bool> mInitialized{false};

    // Thread index: set via thread_local, queried by getThreadIndex()
    static thread_local int tThreadIndex;

    // Task queues — protected by mQueueMutex
    mutable std::mutex mQueueMutex;
    std::condition_variable mQueueCV;          // workers wait here
    std::deque<Task *> mHighQueue;
    std::deque<Task *> mLowQueue;

    // Task/group ownership — protected by mQueueMutex
    std::unordered_map<TaskID, std::unique_ptr<Task>> mTasks;
    std::unordered_map<GroupID, std::unique_ptr<Group>> mGroups;

    // Completion notification — waitFor() and waitForAll() wait here
    mutable std::mutex mCompletionMutex;
    std::condition_variable mCompletionCV;

    // Monotonic ID counters
    std::atomic<int64_t> mNextTaskId{1};
    std::atomic<int64_t> mNextGroupId{1};

    // Outstanding task count for waitForAll()
    std::atomic<int64_t> mOutstandingTasks{0};

    // Low-priority thread limiting — protected by mQueueMutex
    int mMaxLowPriorityThreads = 1;
    int mActiveLowPriorityThreads = 0;
};

} // namespace Darkness

#endif // __WORKER_THREAD_POOL_H
