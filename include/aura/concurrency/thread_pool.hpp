/**
 * @file thread_pool.hpp
 * @brief High-performance thread pool with work-stealing scheduler
 *
 * Provides a flexible thread pool implementation with:
 * - Work-stealing for load balancing
 * - Priority-based task scheduling
 * - Future-based result retrieval
 * - Graceful shutdown
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/concurrency/lock_free_ring_buffer.hpp>
#include <aura/core/logging.hpp>
#include <aura/core/memory_pool.hpp>

#include <atomic>
#include <concepts>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <vector>

namespace aura {

/**
 * @brief Task priority levels
 */
enum class TaskPriority : int { Low = 0, Normal = 1, High = 2, Critical = 3 };

/**
 * @brief Wrapped task with priority
 */
class PrioritizedTask {
public:
    PrioritizedTask() = default;

    template <typename F>
    PrioritizedTask(F&& func, TaskPriority priority)
        : task_(std::forward<F>(func)), priority_(priority) {}

    void operator()() {
        if (task_) {
            task_();
        }
    }

    [[nodiscard]] TaskPriority priority() const noexcept { return priority_; }

    [[nodiscard]] explicit operator bool() const noexcept { return static_cast<bool>(task_); }

private:
    std::function<void()> task_;
    TaskPriority priority_{TaskPriority::Normal};
};

/**
 * @brief Thread-safe task queue with priority support
 */
class TaskQueue {
public:
    TaskQueue() = default;

    /// Push a task to the queue
    void push(PrioritizedTask&& task) {
        std::lock_guard lock(mutex_);
        tasks_.push_back(std::move(task));
        // Sort by priority (higher priority first)
        // For simplicity, we just keep highest priority at back for pop
    }

    /// Try to pop a task from the queue
    [[nodiscard]] std::optional<PrioritizedTask> tryPop() {
        std::lock_guard lock(mutex_);
        if (tasks_.empty()) {
            return std::nullopt;
        }

        // Find highest priority task
        auto it = std::max_element(tasks_.begin(), tasks_.end(),
                                   [](const PrioritizedTask& a, const PrioritizedTask& b) {
                                       return a.priority() < b.priority();
                                   });

        if (it != tasks_.end()) {
            PrioritizedTask task = std::move(*it);
            tasks_.erase(it);
            return task;
        }
        return std::nullopt;
    }

    /// Try to steal a task (from the back, lower priority first)
    [[nodiscard]] std::optional<PrioritizedTask> trySteal() {
        std::lock_guard lock(mutex_);
        if (tasks_.empty()) {
            return std::nullopt;
        }

        // Steal lowest priority task
        auto it = std::min_element(tasks_.begin(), tasks_.end(),
                                   [](const PrioritizedTask& a, const PrioritizedTask& b) {
                                       return a.priority() < b.priority();
                                   });

        if (it != tasks_.end()) {
            PrioritizedTask task = std::move(*it);
            tasks_.erase(it);
            return task;
        }
        return std::nullopt;
    }

    /// Check if queue is empty
    [[nodiscard]] bool empty() const {
        std::lock_guard lock(mutex_);
        return tasks_.empty();
    }

    /// Get queue size
    [[nodiscard]] std::size_t size() const {
        std::lock_guard lock(mutex_);
        return tasks_.size();
    }

    /// Clear all tasks
    void clear() {
        std::lock_guard lock(mutex_);
        tasks_.clear();
    }

private:
    mutable std::mutex mutex_;
    std::deque<PrioritizedTask> tasks_;
};

/**
 * @brief High-performance thread pool with work-stealing
 *
 * Features:
 * - Work-stealing scheduler for load balancing
 * - Per-thread task queues
 * - Priority-based task execution
 * - Graceful shutdown with task completion
 */
class ThreadPool {
public:
    /**
     * @brief Construct a thread pool
     *
     * @param num_threads Number of worker threads (0 = hardware concurrency)
     */
    explicit ThreadPool(std::size_t num_threads = 0)
        : num_threads_(num_threads == 0 ? std::thread::hardware_concurrency() : num_threads),
          task_queues_(num_threads_), running_(true), pending_tasks_(0) {
        AURA_LOG_INFO("ThreadPool: Starting {} worker threads", num_threads_);

        workers_.reserve(num_threads_);
        for (std::size_t i = 0; i < num_threads_; ++i) {
            workers_.emplace_back([this, i](std::stop_token stop_token) {
                workerLoop(static_cast<unsigned int>(i), stop_token);
            });
        }
    }

    // Non-copyable, non-movable
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    /**
     * @brief Destructor - waits for all tasks to complete
     */
    ~ThreadPool() { shutdown(); }

    /**
     * @brief Submit a task for execution
     *
     * @tparam F Callable type
     * @tparam Args Argument types
     * @param priority Task priority
     * @param func Function to execute
     * @param args Arguments to pass to the function
     * @return Future for the result
     */
    template <typename F, typename... Args>
        requires std::invocable<F, Args...>
    [[nodiscard]] auto submit(TaskPriority priority, F&& func, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>> {
        using ReturnType = std::invoke_result_t<F, Args...>;

        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(func), std::forward<Args>(args)...));

        std::future<ReturnType> result = task->get_future();

        {
            if (!running_.load(std::memory_order_acquire)) {
                throw std::runtime_error("Cannot submit to stopped ThreadPool");
            }

            pending_tasks_.fetch_add(1, std::memory_order_relaxed);

            // Submit to least loaded queue (simple round-robin for now)
            auto queue_idx = next_queue_.fetch_add(1, std::memory_order_relaxed) % num_threads_;
            task_queues_[queue_idx].push(PrioritizedTask([task]() { (*task)(); }, priority));
        }

        // Wake up a worker
        condition_.notify_one();

        return result;
    }

    /**
     * @brief Submit a task with normal priority
     */
    template <typename F, typename... Args>
        requires std::invocable<F, Args...>
    [[nodiscard]] auto submit(F&& func, Args&&... args) {
        return submit(TaskPriority::Normal, std::forward<F>(func), std::forward<Args>(args)...);
    }

    /**
     * @brief Submit a task without returning a future (fire-and-forget)
     */
    template <typename F, typename... Args>
        requires std::invocable<F, Args...>
    void execute(TaskPriority priority, F&& func, Args&&... args) {
        auto wrapped = [f = std::forward<F>(func), ... a = std::forward<Args>(args)]() mutable {
            std::invoke(std::move(f), std::move(a)...);
        };

        {
            if (!running_.load(std::memory_order_acquire)) {
                throw std::runtime_error("Cannot execute on stopped ThreadPool");
            }

            pending_tasks_.fetch_add(1, std::memory_order_relaxed);

            auto queue_idx = next_queue_.fetch_add(1, std::memory_order_relaxed) % num_threads_;
            task_queues_[queue_idx].push(PrioritizedTask(std::move(wrapped), priority));
        }

        condition_.notify_one();
    }

    /**
     * @brief Wait for all pending tasks to complete
     */
    void waitAll() {
        std::unique_lock lock(wait_mutex_);
        wait_condition_.wait(
            lock, [this] { return pending_tasks_.load(std::memory_order_acquire) == 0; });
    }

    /**
     * @brief Shutdown the thread pool
     *
     * @param wait_for_tasks If true, wait for pending tasks to complete
     */
    void shutdown(bool wait_for_tasks = true) {
        if (!running_.exchange(false, std::memory_order_acq_rel)) {
            return;  // Already stopped
        }

        AURA_LOG_INFO("ThreadPool: Shutting down...");

        // If wait_for_tasks is true, wait for all pending tasks to complete
        if (wait_for_tasks) {
            waitAll();
        }

        // Wake all workers
        condition_.notify_all();

        // Request stop for all workers
        for (auto& worker : workers_) {
            worker.request_stop();
        }

        // Wait for workers to finish
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        AURA_LOG_INFO("ThreadPool: Shutdown complete");
    }

    /// Get the number of worker threads
    [[nodiscard]] std::size_t numThreads() const noexcept { return num_threads_; }

    /// Get the number of pending tasks
    [[nodiscard]] std::size_t pendingTasks() const noexcept {
        return pending_tasks_.load(std::memory_order_relaxed);
    }

    /// Check if the pool is running
    [[nodiscard]] bool isRunning() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    /// Get queue statistics
    struct Stats {
        std::size_t total_queued;
        std::size_t pending_tasks;
        std::vector<std::size_t> queue_sizes;
    };

    [[nodiscard]] Stats stats() const {
        Stats s;
        s.pending_tasks = pending_tasks_.load(std::memory_order_relaxed);
        s.total_queued = 0;
        s.queue_sizes.reserve(num_threads_);
        for (const auto& queue : task_queues_) {
            auto size = queue.size();
            s.queue_sizes.push_back(size);
            s.total_queued += size;
        }
        return s;
    }

private:
    /**
     * @brief Worker thread main loop
     */
    void workerLoop(unsigned int worker_id, std::stop_token stop_token) {
        // Thread-local random generator for work stealing
        thread_local std::mt19937 rng(std::hash<std::thread::id>{}(std::this_thread::get_id()));

        while (!stop_token.stop_requested()) {
            std::optional<PrioritizedTask> task;

            // Try to get task from own queue first
            task = task_queues_[worker_id].tryPop();

            // If no task, try to steal from another queue
            if (!task) {
                for (std::size_t attempts = 0; attempts < num_threads_ && !task; ++attempts) {
                    auto victim = rng() % num_threads_;
                    if (victim != worker_id) {
                        task = task_queues_[victim].trySteal();
                    }
                }
            }

            if (task) {
                try {
                    (*task)();
                } catch (const std::exception& e) {
                    AURA_LOG_ERROR("ThreadPool: Task threw exception: {}", e.what());
                } catch (...) {
                    AURA_LOG_ERROR("ThreadPool: Task threw unknown exception");
                }

                auto remaining = pending_tasks_.fetch_sub(1, std::memory_order_relaxed) - 1;
                if (remaining == 0) {
                    wait_condition_.notify_all();
                }
            } else {
                // No work available, wait for notification
                std::unique_lock lock(mutex_);
                condition_.wait_for(lock, std::chrono::milliseconds(10), [&] {
                    return stop_token.stop_requested() || !task_queues_[worker_id].empty();
                });
            }
        }
    }

    const std::size_t num_threads_;
    std::vector<TaskQueue> task_queues_;
    std::vector<std::jthread> workers_;

    std::atomic<bool> running_{true};
    std::atomic<std::size_t> pending_tasks_{0};
    std::atomic<std::size_t> next_queue_{0};

    std::mutex mutex_;
    std::condition_variable condition_;

    std::mutex wait_mutex_;
    std::condition_variable wait_condition_;
};

/**
 * @brief Get the global thread pool instance
 *
 * @return Reference to the singleton thread pool
 */
inline ThreadPool& globalThreadPool() {
    static ThreadPool pool;
    return pool;
}

/**
 * @brief Submit a task to the global thread pool
 */
template <typename F, typename... Args>
    requires std::invocable<F, Args...>
auto asyncSubmit(F&& func, Args&&... args) {
    return globalThreadPool().submit(std::forward<F>(func), std::forward<Args>(args)...);
}

/**
 * @brief Submit a prioritized task to the global thread pool
 */
template <typename F, typename... Args>
    requires std::invocable<F, Args...>
auto asyncSubmit(TaskPriority priority, F&& func, Args&&... args) {
    return globalThreadPool().submit(priority, std::forward<F>(func), std::forward<Args>(args)...);
}

}  // namespace aura
