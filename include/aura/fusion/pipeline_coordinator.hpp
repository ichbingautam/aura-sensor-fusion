/**
 * @file pipeline_coordinator.hpp
 * @brief Coordinator for the fusion pipeline
 *
 * Manages the lifecycle and execution of fusion nodes.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/concurrency/thread_pool.hpp>
#include <aura/core/logging.hpp>
#include <aura/fusion/fusion_node.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

namespace aura {

/**
 * @brief Coordinates the execution of valid FusionNodes
 */
class PipelineCoordinator {
public:
    explicit PipelineCoordinator(ThreadPool& thread_pool)
        : thread_pool_(thread_pool), running_(false) {}

    ~PipelineCoordinator() { stop(); }

    /// Register a node with the coordinator
    void registerNode(std::shared_ptr<FusionNodeBase> node) {
        std::lock_guard lock(mutex_);
        nodes_.push_back(std::move(node));
    }

    /// Start the pipeline coordinator
    bool start() {
        if (running_.exchange(true)) {
            return false;  // Already running
        }

        AURA_LOG_INFO("PipelineCoordinator: Starting...");

        // Initialize and start all nodes
        {
            std::lock_guard lock(mutex_);
            for (auto& node : nodes_) {
                if (!node->initialize()) {
                    AURA_LOG_ERROR("Failed to initialize node: {}", node->name());
                    // Continue trying others? Or fail?
                    // For now, log and continue
                }
                node->start();
            }
        }

        // Start coordination loop
        coordinator_thread_ = std::thread(&PipelineCoordinator::loop, this);
        return true;
    }

    /// Stop the pipeline coordinator
    void stop() {
        if (!running_.exchange(false)) {
            return;  // Already stopped
        }

        AURA_LOG_INFO("PipelineCoordinator: Stopping...");

        if (coordinator_thread_.joinable()) {
            coordinator_thread_.join();
        }

        // Stop all nodes
        {
            std::lock_guard lock(mutex_);
            for (auto& node : nodes_) {
                node->stop();
            }
        }
    }

private:
    void loop() {
        while (running_) {
            bool busy = false;

            // Snapshot of nodes to avoid holding lock too long
            // (Assuming nodes list doesn't change often after start,
            // but for safety we should probably lock or copy)
            std::vector<std::shared_ptr<FusionNodeBase>> active_nodes;
            {
                std::lock_guard lock(mutex_);
                active_nodes = nodes_;
            }

            for (const auto& node : active_nodes) {
                if (node->hasPendingInput()) {
                    // Submit a task to process a batch of inputs
                    thread_pool_.submit(TaskPriority::Normal, [node]() {
                        // Process up to batch_size items to amortize task overhead
                        // but yield eventually to ensure fairness
                        constexpr int MAX_BATCH_SIZE = 16;
                        int processed = 0;
                        while (processed < MAX_BATCH_SIZE && node->processNext()) {
                            processed++;
                        }
                    });
                    busy = true;
                }
            }

            // Sleep to avoid busy loop if no work was scheduled
            if (!busy) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else {
                // Even if busy, yield slightly to allow checking stop_requested or allow other
                // threads
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    }

    ThreadPool& thread_pool_;
    std::vector<std::shared_ptr<FusionNodeBase>> nodes_;
    std::mutex mutex_;
    std::atomic<bool> running_;
    std::thread coordinator_thread_;
};

}  // namespace aura
