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

#include "WorkerThreadPool.h"
#include "logger.h"

#include <algorithm>
#include <cassert>

// Safe logging: only log if the Logger singleton exists.
// In test environments or early startup, Logger may not be initialized.
#define POOL_LOG_INFO(...)                                                     \
    do {                                                                        \
        if (::Darkness::Logger::getSingletonPtr())                              \
            LOG_INFO(__VA_ARGS__);                                              \
    } while (0)

#define POOL_LOG_ERROR(...)                                                    \
    do {                                                                        \
        if (::Darkness::Logger::getSingletonPtr())                              \
            LOG_ERROR(__VA_ARGS__);                                             \
    } while (0)

#if defined(__APPLE__)
#include <pthread.h>
#endif

namespace Darkness {

// Thread-local worker index: -1 for non-pool threads
thread_local int WorkerThreadPool::tThreadIndex = -1;

// ─── Construction / Destruction ──────────────────────────────────────────────

WorkerThreadPool::WorkerThreadPool() = default;

WorkerThreadPool::~WorkerThreadPool() {
    if (mInitialized.load(std::memory_order_acquire)) {
        shutdown();
    }
}

// ─── Initialization ─────────────────────────────────────────────────────────

void WorkerThreadPool::init(int threadCount, int reservedThreads,
                            float lowPriorityRatio) {
    assert(!mInitialized.load(std::memory_order_relaxed)
           && "WorkerThreadPool::init() called twice");

    int hwThreads = static_cast<int>(std::thread::hardware_concurrency());

    // Auto thread count: leave room for audio + main thread
    if (threadCount < 0) {
        threadCount = std::max(2, hwThreads - reservedThreads);
    }

    // Clamp low-priority ratio and compute max low-priority threads
    lowPriorityRatio = std::max(0.0f, std::min(1.0f, lowPriorityRatio));
    mMaxLowPriorityThreads = std::max(1, static_cast<int>(
        threadCount * lowPriorityRatio));

    POOL_LOG_INFO("WorkerThreadPool: starting %d threads (hw=%d, reserved=%d, "
             "lowPriMax=%d)", threadCount, hwThreads, reservedThreads,
             mMaxLowPriorityThreads);

    mShutdown.store(false, std::memory_order_relaxed);
    mInitialized.store(true, std::memory_order_release);

    mThreads.reserve(threadCount);
    for (int i = 0; i < threadCount; ++i) {
        mThreads.emplace_back(&WorkerThreadPool::workerMain, this, i);
    }
}

void WorkerThreadPool::shutdown() {
    // Atomically check and clear mInitialized to prevent double-shutdown
    bool expected = true;
    if (!mInitialized.compare_exchange_strong(expected, false,
                                               std::memory_order_acq_rel)) {
        return; // Already shut down or never initialized
    }

    POOL_LOG_INFO("WorkerThreadPool: shutting down...");

    // Signal all workers to exit and wake them
    mShutdown.store(true, std::memory_order_release);
    mQueueCV.notify_all();

    // Join all worker threads — ensures in-flight tasks complete before cleanup
    for (auto &t : mThreads) {
        if (t.joinable())
            t.join();
    }
    mThreads.clear();

    // Clean up any remaining tasks/groups (safe: all workers have exited)
    {
        std::lock_guard<std::mutex> lock(mQueueMutex);
        mHighQueue.clear();
        mLowQueue.clear();
        mTasks.clear();
        mGroups.clear();
    }

    mOutstandingTasks.store(0, std::memory_order_release);

    // Wake any threads blocked in waitFor/waitForAll so they can observe
    // the cleared state and return
    {
        std::lock_guard<std::mutex> lock(mCompletionMutex);
    }
    mCompletionCV.notify_all();

    POOL_LOG_INFO("WorkerThreadPool: shutdown complete.");
}

bool WorkerThreadPool::isRunning() const {
    return mInitialized.load(std::memory_order_acquire);
}

int WorkerThreadPool::getThreadCount() const {
    // Safe: mThreads is only modified in init()/shutdown() which are
    // main-thread operations. getThreadCount() is read-only.
    return static_cast<int>(mThreads.size());
}

// ─── Single Task Submission ──────────────────────────────────────────────────

TaskID WorkerThreadPool::submit(std::function<void()> work, Priority priority) {
    TaskID id = mNextTaskId.fetch_add(1, std::memory_order_relaxed);

    auto task = std::make_unique<Task>();
    task->id = id;
    task->work = std::move(work);
    task->priority = priority;

    {
        std::lock_guard<std::mutex> lock(mQueueMutex);

        // Reject tasks during shutdown — the task would never execute
        if (mShutdown.load(std::memory_order_acquire)) {
            return INVALID_TASK_ID;
        }

        mOutstandingTasks.fetch_add(1, std::memory_order_release);

        Task *rawPtr = task.get();
        mTasks.emplace(id, std::move(task));

        if (priority == Priority::High) {
            mHighQueue.push_back(rawPtr);
        } else {
            mLowQueue.push_back(rawPtr);
        }
    }

    mQueueCV.notify_one();
    return id;
}

// ─── Task Completion ─────────────────────────────────────────────────────────

bool WorkerThreadPool::isComplete(TaskID id) const {
    if (id == INVALID_TASK_ID)
        return true;

    std::lock_guard<std::mutex> lock(mQueueMutex);
    auto it = mTasks.find(id);
    if (it == mTasks.end())
        return true; // Unknown/already-cleaned-up ID — treat as complete

    return it->second->completed.load(std::memory_order_acquire);
}

void WorkerThreadPool::waitFor(TaskID id) {
    if (id == INVALID_TASK_ID)
        return;

    std::unique_lock<std::mutex> lock(mCompletionMutex);
    mCompletionCV.wait(lock, [this, id] {
        return isComplete(id);
    });
}

void WorkerThreadPool::waitForAll() {
    std::unique_lock<std::mutex> lock(mCompletionMutex);
    mCompletionCV.wait(lock, [this] {
        return mOutstandingTasks.load(std::memory_order_acquire) <= 0;
    });
}

void WorkerThreadPool::pruneCompleted() {
    std::lock_guard<std::mutex> lock(mQueueMutex);

    // Remove completed tasks
    for (auto it = mTasks.begin(); it != mTasks.end(); ) {
        if (it->second->completed.load(std::memory_order_acquire)) {
            it = mTasks.erase(it);
        } else {
            ++it;
        }
    }

    // Remove fully completed groups
    for (auto it = mGroups.begin(); it != mGroups.end(); ) {
        if (it->second->completedCount.load(std::memory_order_acquire)
            >= it->second->totalCount) {
            it = mGroups.erase(it);
        } else {
            ++it;
        }
    }
}

// ─── Group Task Submission ───────────────────────────────────────────────────

GroupID WorkerThreadPool::submitGroup(std::function<void(uint32_t)> work,
                                      uint32_t count, Priority priority) {
    GroupID gid = mNextGroupId.fetch_add(1, std::memory_order_relaxed);

    if (count == 0) {
        // Empty group — immediately complete
        auto group = std::make_unique<Group>();
        group->id = gid;
        group->totalCount = 0;
        group->completedCount.store(0, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(mQueueMutex);
        mGroups.emplace(gid, std::move(group));
        return gid;
    }

    // Create the group
    auto group = std::make_unique<Group>();
    group->id = gid;
    group->totalCount = count;
    group->completedCount.store(0, std::memory_order_relaxed);
    group->work = work; // Shared callable — captured by value

    {
        std::lock_guard<std::mutex> lock(mQueueMutex);

        // Reject during shutdown
        if (mShutdown.load(std::memory_order_acquire)) {
            return INVALID_GROUP_ID;
        }

        mOutstandingTasks.fetch_add(count, std::memory_order_release);

        Group *groupPtr = group.get();
        mGroups.emplace(gid, std::move(group));

        // Create one task per element
        for (uint32_t i = 0; i < count; ++i) {
            TaskID tid = mNextTaskId.fetch_add(1, std::memory_order_relaxed);
            auto task = std::make_unique<Task>();
            task->id = tid;
            task->priority = priority;
            task->group = groupPtr; // Direct pointer, no map lookup needed

            // Capture index and group pointer for the work lambda.
            // groupPtr remains valid: Group is owned by mGroups, only freed
            // in shutdown() which joins all workers first.
            task->work = [groupPtr, i]() {
                groupPtr->work(i);
            };

            Task *rawPtr = task.get();
            mTasks.emplace(tid, std::move(task));

            if (priority == Priority::High) {
                mHighQueue.push_back(rawPtr);
            } else {
                mLowQueue.push_back(rawPtr);
            }
        }
    }

    // Wake enough workers for the batch
    mQueueCV.notify_all();
    return gid;
}

bool WorkerThreadPool::isGroupComplete(GroupID id) const {
    if (id == INVALID_GROUP_ID)
        return true;

    std::lock_guard<std::mutex> lock(mQueueMutex);
    auto it = mGroups.find(id);
    if (it == mGroups.end())
        return true;

    const auto &group = it->second;
    return group->completedCount.load(std::memory_order_acquire)
           >= group->totalCount;
}

void WorkerThreadPool::waitForGroup(GroupID id) {
    if (id == INVALID_GROUP_ID)
        return;

    std::unique_lock<std::mutex> lock(mCompletionMutex);
    mCompletionCV.wait(lock, [this, id] {
        return isGroupComplete(id);
    });
}

uint32_t WorkerThreadPool::getGroupProgress(GroupID id) const {
    if (id == INVALID_GROUP_ID)
        return 0;

    std::lock_guard<std::mutex> lock(mQueueMutex);
    auto it = mGroups.find(id);
    if (it == mGroups.end())
        return 0;

    return it->second->completedCount.load(std::memory_order_acquire);
}

// ─── Thread Identity ─────────────────────────────────────────────────────────

int WorkerThreadPool::getThreadIndex() const {
    return tThreadIndex;
}

// ─── Worker Thread ───────────────────────────────────────────────────────────

WorkerThreadPool::Task *WorkerThreadPool::popTask(bool allowLowPriority) {
    // Caller must hold mQueueMutex

    // Always prefer high-priority work
    if (!mHighQueue.empty()) {
        Task *task = mHighQueue.front();
        mHighQueue.pop_front();
        return task;
    }

    // Low-priority: only if allowed and under the concurrency limit
    if (allowLowPriority && !mLowQueue.empty()) {
        Task *task = mLowQueue.front();
        mLowQueue.pop_front();
        return task;
    }

    return nullptr;
}

void WorkerThreadPool::workerMain(int threadIndex) {
    // Set thread-local index for getThreadIndex()
    tThreadIndex = threadIndex;

    // Name the thread for debugger visibility
#if defined(__APPLE__)
    {
        char name[16];
        snprintf(name, sizeof(name), "DkPool-%d", threadIndex);
        pthread_setname_np(name);
    }
#elif defined(__linux__)
    {
        char name[16];
        snprintf(name, sizeof(name), "DkPool-%d", threadIndex);
        pthread_setname_np(pthread_self(), name);
    }
#endif

    while (true) {
        Task *task = nullptr;
        bool isLowPriority = false;

        {
            std::unique_lock<std::mutex> lock(mQueueMutex);

            // Wait predicate includes low-priority availability check to
            // prevent spin-looping when only low-pri tasks are queued but
            // the concurrency limit is reached.
            mQueueCV.wait(lock, [this] {
                if (mShutdown.load(std::memory_order_acquire))
                    return true;
                if (!mHighQueue.empty())
                    return true;
                if (!mLowQueue.empty() &&
                    mActiveLowPriorityThreads < mMaxLowPriorityThreads)
                    return true;
                return false;
            });

            if (mShutdown.load(std::memory_order_acquire)
                && mHighQueue.empty() && mLowQueue.empty()) {
                break;
            }

            // Check low-priority allowance under the lock
            bool canRunLow =
                mActiveLowPriorityThreads < mMaxLowPriorityThreads;

            task = popTask(canRunLow);

            if (!task)
                continue; // Shutdown with empty queues, loop will exit

            // Track low-priority thread usage under the lock
            isLowPriority = (task->priority == Priority::Low);
            if (isLowPriority) {
                ++mActiveLowPriorityThreads;
            }
        }

        // Execute the task outside the queue lock.
        // Catch exceptions to prevent worker thread death and ensure
        // the task is properly marked complete even on failure.
        try {
            task->work();
        } catch (...) {
            POOL_LOG_ERROR("WorkerThreadPool: task %lld threw an exception",
                           static_cast<long long>(task->id));
        }

        // Clear the work function to release any captured resources promptly
        task->work = nullptr;

        // Mark task complete (release ensures work() side effects are
        // visible to any thread that observes completed==true)
        task->completed.store(true, std::memory_order_release);

        // Update group progress via direct pointer (no map lookup needed)
        if (task->group) {
            task->group->completedCount.fetch_add(1, std::memory_order_release);
        }

        // Release low-priority slot under the lock and wake blocked workers
        // that may have been waiting for a low-priority slot to open
        if (isLowPriority) {
            {
                std::lock_guard<std::mutex> lock(mQueueMutex);
                --mActiveLowPriorityThreads;
            }
            // Wake workers that may be blocked waiting for low-pri availability
            mQueueCV.notify_one();
        }

        // Decrement outstanding count and notify completion waiters.
        // Lock mCompletionMutex briefly to establish happens-before with
        // the CV wait in waitFor()/waitForAll() — textbook pattern.
        mOutstandingTasks.fetch_sub(1, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(mCompletionMutex);
        }
        mCompletionCV.notify_all();
    }
}

} // namespace Darkness
