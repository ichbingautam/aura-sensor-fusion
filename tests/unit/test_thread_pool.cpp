/**
 * @file test_thread_pool.cpp
 * @brief Unit tests for thread pool
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#include <aura/concurrency/thread_pool.hpp>

#include <atomic>
#include <chrono>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

namespace aura::test {

class ThreadPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Disable logging for tests
        Logger::instance().setLevel(LogLevel::Off);
    }
};

TEST_F(ThreadPoolTest, Construction) {
    ThreadPool pool(4);
    EXPECT_EQ(pool.numThreads(), 4);
    EXPECT_TRUE(pool.isRunning());
    EXPECT_EQ(pool.pendingTasks(), 0);
}

TEST_F(ThreadPoolTest, DefaultConstruction) {
    ThreadPool pool;
    EXPECT_GE(pool.numThreads(), 1);
    EXPECT_TRUE(pool.isRunning());
}

TEST_F(ThreadPoolTest, SimpleTask) {
    ThreadPool pool(2);

    auto future = pool.submit([]() { return 42; });

    EXPECT_EQ(future.get(), 42);
}

TEST_F(ThreadPoolTest, MultipleTasksWithResults) {
    ThreadPool pool(4);

    std::vector<std::future<int>> futures;
    for (int i = 0; i < 100; ++i) {
        futures.push_back(pool.submit([i]() { return i * 2; }));
    }

    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(futures[i].get(), i * 2);
    }
}

TEST_F(ThreadPoolTest, VoidTasks) {
    ThreadPool pool(2);
    std::atomic<int> counter{0};

    std::vector<std::future<void>> futures;
    for (int i = 0; i < 50; ++i) {
        futures.push_back(pool.submit([&counter]() { counter.fetch_add(1); }));
    }

    for (auto& f : futures) {
        f.get();
    }

    EXPECT_EQ(counter.load(), 50);
}

TEST_F(ThreadPoolTest, PriorityTasks) {
    ThreadPool pool(1);  // Single thread to observe ordering

    std::vector<int> execution_order;
    std::mutex order_mutex;

    // Submit low priority first
    auto low = pool.submit(TaskPriority::Low, [&]() {
        std::lock_guard lock(order_mutex);
        execution_order.push_back(1);
    });

    // Give the low priority task time to be queued but not necessarily started
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Submit high priority
    auto high = pool.submit(TaskPriority::High, [&]() {
        std::lock_guard lock(order_mutex);
        execution_order.push_back(3);
    });

    low.get();
    high.get();

    // Both tasks should have executed
    EXPECT_EQ(execution_order.size(), 2);
}

TEST_F(ThreadPoolTest, WaitAll) {
    ThreadPool pool(4);
    std::atomic<int> counter{0};

    for (int i = 0; i < 100; ++i) {
        pool.execute(TaskPriority::Normal, [&counter]() {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            counter.fetch_add(1);
        });
    }

    pool.waitAll();
    EXPECT_EQ(counter.load(), 100);
}

TEST_F(ThreadPoolTest, Shutdown) {
    auto pool = std::make_unique<ThreadPool>(4);
    std::atomic<int> counter{0};

    for (int i = 0; i < 50; ++i) {
        pool->execute(TaskPriority::Normal, [&counter]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            counter.fetch_add(1);
        });
    }

    pool->shutdown(true);  // Wait for tasks
    EXPECT_FALSE(pool->isRunning());
    EXPECT_EQ(counter.load(), 50);
}

TEST_F(ThreadPoolTest, Stats) {
    ThreadPool pool(2);

    // Submit some tasks
    for (int i = 0; i < 10; ++i) {
        pool.submit([]() { return 0; });
    }

    pool.waitAll();

    auto stats = pool.stats();
    EXPECT_EQ(stats.pending_tasks, 0);
}

TEST_F(ThreadPoolTest, ExceptionHandling) {
    ThreadPool pool(2);
    Logger::instance().setLevel(LogLevel::Off);  // Suppress error logs

    // Task that throws
    auto future = pool.submit([]() -> int { throw std::runtime_error("Test exception"); });

    EXPECT_THROW(future.get(), std::runtime_error);
}

TEST_F(ThreadPoolTest, TaskWithArguments) {
    ThreadPool pool(2);

    auto future = pool.submit([](int a, int b, int c) { return a + b + c; }, 10, 20, 30);

    EXPECT_EQ(future.get(), 60);
}

TEST_F(ThreadPoolTest, GlobalThreadPool) {
    auto future = asyncSubmit([]() { return 123; });
    EXPECT_EQ(future.get(), 123);
}

TEST_F(ThreadPoolTest, StressTest) {
    ThreadPool pool(8);
    constexpr int kNumTasks = 10000;
    std::atomic<int> counter{0};

    for (int i = 0; i < kNumTasks; ++i) {
        pool.execute(TaskPriority::Normal, [&counter]() { counter.fetch_add(1); });
    }

    pool.waitAll();
    EXPECT_EQ(counter.load(), kNumTasks);
}

}  // namespace aura::test
