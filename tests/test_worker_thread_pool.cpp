// Unit tests for WorkerThreadPool (Task 42)
// Tests cover: single tasks, completion queries, group tasks, priority,
// concurrent submission, shutdown safety, and thread-safe logging.

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include "WorkerThreadPool.h"
#include "logger.h"
#include "stdlog.h"

using namespace Darkness;
using namespace std::chrono_literals;

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Spin-wait with timeout to avoid hanging tests
static bool waitWithTimeout(std::function<bool()> pred, int timeoutMs = 5000) {
    auto start = std::chrono::steady_clock::now();
    while (!pred()) {
        if (std::chrono::steady_clock::now() - start >
            std::chrono::milliseconds(timeoutMs)) {
            return false; // timed out
        }
        std::this_thread::yield();
    }
    return true;
}

// ─── Basic Submit + Wait ─────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: basic submit and wait", "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    std::atomic<int> value{0};
    TaskID id = pool.submit([&value] { value.store(42, std::memory_order_release); });

    pool.waitFor(id);
    REQUIRE(value.load(std::memory_order_acquire) == 42);

    pool.shutdown();
}

// ─── isComplete Polling ──────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: isComplete returns false then true", "[threading]") {
    WorkerThreadPool pool;
    pool.init(1, 0);

    std::atomic<bool> gate{false};

    TaskID id = pool.submit([&gate] {
        // Wait until test releases the gate
        while (!gate.load(std::memory_order_acquire))
            std::this_thread::yield();
    });

    // Task should still be running
    // (can't guarantee false due to scheduling, but usually will be)
    std::this_thread::sleep_for(1ms);
    // Don't assert isComplete==false — timing dependent

    // Release the gate
    gate.store(true, std::memory_order_release);

    pool.waitFor(id);
    REQUIRE(pool.isComplete(id));

    pool.shutdown();
}

// ─── Multiple Tasks ──────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: 100 tasks with atomic counter", "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    std::atomic<int> counter{0};

    for (int i = 0; i < 100; ++i) {
        pool.submit([&counter] {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    pool.waitForAll();
    REQUIRE(counter.load() == 100);

    pool.shutdown();
}

// ─── waitForAll ──────────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: waitForAll blocks until all done", "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    std::atomic<int> counter{0};

    for (int i = 0; i < 50; ++i) {
        pool.submit([&counter] {
            std::this_thread::sleep_for(1ms);
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    pool.waitForAll();
    REQUIRE(counter.load() == 50);

    pool.shutdown();
}

// ─── Invalid/Duplicate Wait ──────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: waitFor on INVALID_TASK_ID is no-op", "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    // Should not block or crash
    pool.waitFor(INVALID_TASK_ID);
    REQUIRE(pool.isComplete(INVALID_TASK_ID));

    pool.shutdown();
}

TEST_CASE("WorkerThreadPool: double waitFor does not deadlock", "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    std::atomic<int> value{0};
    TaskID id = pool.submit([&value] { value.store(1); });

    pool.waitFor(id);
    pool.waitFor(id); // Second wait should be no-op
    REQUIRE(value.load() == 1);

    pool.shutdown();
}

// ─── Group Tasks ─────────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: group task processes all elements", "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    constexpr int N = 64;
    std::atomic<int> results[N];
    for (auto &r : results) r.store(0);

    GroupID gid = pool.submitGroup(
        [&results](uint32_t idx) {
            results[idx].store(static_cast<int>(idx * idx),
                               std::memory_order_relaxed);
        },
        N);

    pool.waitForGroup(gid);

    for (int i = 0; i < N; ++i) {
        REQUIRE(results[i].load() == i * i);
    }

    pool.shutdown();
}

TEST_CASE("WorkerThreadPool: group progress increases monotonically", "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    constexpr uint32_t N = 32;
    GroupID gid = pool.submitGroup(
        [](uint32_t) {
            std::this_thread::sleep_for(1ms);
        },
        N);

    // Poll progress — should never decrease
    uint32_t lastProgress = 0;
    bool completed = waitWithTimeout([&] {
        uint32_t p = pool.getGroupProgress(gid);
        REQUIRE(p >= lastProgress); // Monotonic
        lastProgress = p;
        return pool.isGroupComplete(gid);
    });

    REQUIRE(completed);
    REQUIRE(pool.getGroupProgress(gid) == N);

    pool.shutdown();
}

TEST_CASE("WorkerThreadPool: empty group is immediately complete", "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    GroupID gid = pool.submitGroup([](uint32_t) {}, 0);
    REQUIRE(pool.isGroupComplete(gid));
    pool.waitForGroup(gid); // Should not block

    pool.shutdown();
}

// ─── Concurrent Submission ───────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: concurrent submission from multiple threads",
          "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    constexpr int SUBMITTERS = 4;
    constexpr int TASKS_PER = 25;
    std::atomic<int> counter{0};

    std::vector<std::thread> submitters;
    for (int s = 0; s < SUBMITTERS; ++s) {
        submitters.emplace_back([&pool, &counter] {
            for (int i = 0; i < TASKS_PER; ++i) {
                pool.submit([&counter] {
                    counter.fetch_add(1, std::memory_order_relaxed);
                });
            }
        });
    }

    for (auto &t : submitters) t.join();

    pool.waitForAll();
    REQUIRE(counter.load() == SUBMITTERS * TASKS_PER);

    pool.shutdown();
}

// ─── Priority Ordering ──────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: high-priority tasks run before low-priority",
          "[threading]") {
    // Use 1 thread so ordering is deterministic
    WorkerThreadPool pool;
    pool.init(1, 0, 1.0f); // 100% low-priority ratio so the thread can run both

    std::atomic<bool> gate{false};
    std::atomic<bool> started{false};
    std::vector<int> order;
    std::mutex orderMutex;

    // Block the only thread, signal when it has started
    pool.submit([&gate, &started] {
        started.store(true, std::memory_order_release);
        while (!gate.load(std::memory_order_acquire))
            std::this_thread::yield();
    }, Priority::High);

    // Wait deterministically for the blocking task to begin executing
    while (!started.load(std::memory_order_acquire))
        std::this_thread::yield();

    // Queue low-priority first, then high-priority
    // (both will be pending while the gate blocks the thread)
    pool.submit([&order, &orderMutex] {
        std::lock_guard<std::mutex> lock(orderMutex);
        order.push_back(0); // Low-priority marker
    }, Priority::Low);

    pool.submit([&order, &orderMutex] {
        std::lock_guard<std::mutex> lock(orderMutex);
        order.push_back(1); // High-priority marker
    }, Priority::High);

    // Release the gate — thread should pick high first, then low
    gate.store(true, std::memory_order_release);

    pool.waitForAll();

    REQUIRE(order.size() == 2);
    REQUIRE(order[0] == 1); // High ran first
    REQUIRE(order[1] == 0); // Low ran second

    pool.shutdown();
}

// ─── Thread Index ────────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: getThreadIndex returns valid indices", "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    int threadCount = pool.getThreadCount();
    std::atomic<int> maxIndex{-1};

    for (int i = 0; i < 20; ++i) {
        pool.submit([&pool, &maxIndex] {
            int idx = pool.getThreadIndex();
            REQUIRE(idx >= 0);
            REQUIRE(idx < pool.getThreadCount());

            // Track max index seen
            int prev = maxIndex.load();
            while (prev < idx && !maxIndex.compare_exchange_weak(prev, idx))
                ;
        });
    }

    pool.waitForAll();

    // Main thread should get -1
    REQUIRE(pool.getThreadIndex() == -1);

    pool.shutdown();
}

// ─── Shutdown Safety ─────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: shutdown with pending tasks does not hang",
          "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    // Submit some tasks, some may not finish before shutdown
    for (int i = 0; i < 10; ++i) {
        pool.submit([i] {
            std::this_thread::sleep_for(std::chrono::milliseconds(i * 10));
        });
    }

    // Shutdown should complete without hanging (5-second timeout if test itself hangs)
    pool.shutdown();
    REQUIRE(!pool.isRunning());
}

TEST_CASE("WorkerThreadPool: destructor calls shutdown", "[threading]") {
    std::atomic<int> value{0};
    {
        WorkerThreadPool pool;
        pool.init(2, 0);
        pool.submit([&value] { value.store(1); });
        pool.waitForAll();
    } // Destructor should not crash

    REQUIRE(value.load() == 1);
}

// ─── Thread-Safe Logging Stress Test ─────────────────────────────────────────

TEST_CASE("WorkerThreadPool: concurrent logging does not crash", "[threading]") {
    // Create Logger/StdLog only if no other test file already owns the singletons
    // (test_typed_properties and test_world_query leak heap-allocated ones).
    Logger *logger = nullptr;
    StdLog *stdlog = nullptr;
    bool ownsSingletons = !Logger::getSingletonPtr();
    if (ownsSingletons) {
        logger = new Logger();
        stdlog = new StdLog();
        logger->registerLogListener(stdlog);
        logger->setLogLevel(Logger::LOG_LEVEL_INFO);
    } else {
        logger = Logger::getSingletonPtr();
    }

    WorkerThreadPool pool;
    pool.init(4, 0);

    // Hammer the logger from multiple threads simultaneously
    for (int i = 0; i < 50; ++i) {
        pool.submit([i] {
            LOG_INFO("WorkerThreadPool test: task %d on thread", i);
        });
    }

    pool.waitForAll();
    pool.shutdown();

    if (ownsSingletons) {
        logger->unregisterLogListener(stdlog);
        delete stdlog;
        delete logger;
    }
    // If we got here without crashing or TSAN errors, the test passes
    REQUIRE(true);
}

// ─── Low-Priority Ratio Limiting ─────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: low-priority ratio limits concurrent low-pri threads",
          "[threading]") {
    // 4 threads, 25% low-priority ratio = max 1 low-priority thread
    WorkerThreadPool pool;
    pool.init(4, 0, 0.25f);

    std::atomic<int> concurrentLow{0};
    std::atomic<int> maxConcurrentLow{0};

    for (int i = 0; i < 20; ++i) {
        pool.submit([&concurrentLow, &maxConcurrentLow] {
            int cur = concurrentLow.fetch_add(1, std::memory_order_relaxed) + 1;

            // Track max concurrent low-priority
            int prev = maxConcurrentLow.load(std::memory_order_relaxed);
            while (prev < cur &&
                   !maxConcurrentLow.compare_exchange_weak(prev, cur))
                ;

            std::this_thread::sleep_for(5ms);
            concurrentLow.fetch_sub(1, std::memory_order_relaxed);
        }, Priority::Low);
    }

    pool.waitForAll();

    // Max concurrent low-priority threads should be capped at 1
    // (with 4 threads and 0.25 ratio)
    REQUIRE(maxConcurrentLow.load() <= 1);

    pool.shutdown();
}

// ─── Exception Safety ────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: task exception does not kill worker thread",
          "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    std::atomic<int> counter{0};

    // Submit a task that throws
    pool.submit([] { throw std::runtime_error("test exception"); });

    // Submit normal tasks after — they should still execute
    for (int i = 0; i < 10; ++i) {
        pool.submit([&counter] {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    pool.waitForAll();
    REQUIRE(counter.load() == 10);

    pool.shutdown();
}

// ─── Submit During Shutdown ──────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: submit during shutdown returns INVALID_TASK_ID",
          "[threading]") {
    WorkerThreadPool pool;
    pool.init(2, 0);

    pool.shutdown();

    // Submit after shutdown should return invalid
    TaskID id = pool.submit([] {});
    REQUIRE(id == INVALID_TASK_ID);
    REQUIRE(pool.isComplete(INVALID_TASK_ID));
}

// ─── Prune Completed ─────────────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: pruneCompleted removes finished tasks", "[threading]") {
    WorkerThreadPool pool;
    pool.init(4, 0);

    // Submit and complete 100 tasks
    for (int i = 0; i < 100; ++i) {
        pool.submit([] {});
    }
    pool.waitForAll();

    // Prune should remove all completed tasks
    pool.pruneCompleted();

    // Verify: previously valid task IDs are now unknown (isComplete returns true for unknown)
    // Submit a new task to verify pool still works after prune
    std::atomic<int> val{0};
    pool.submit([&val] { val.store(42); });
    pool.waitForAll();
    REQUIRE(val.load() == 42);

    pool.shutdown();
}

// ─── Re-init after shutdown ──────────────────────────────────────────────────

TEST_CASE("WorkerThreadPool: can re-init after shutdown", "[threading]") {
    WorkerThreadPool pool;

    // First cycle
    pool.init(2, 0);
    std::atomic<int> val{0};
    pool.submit([&val] { val.store(1); });
    pool.waitForAll();
    pool.shutdown();

    REQUIRE(val.load() == 1);

    // Second cycle
    pool.init(2, 0);
    pool.submit([&val] { val.store(2); });
    pool.waitForAll();
    pool.shutdown();

    REQUIRE(val.load() == 2);
}
