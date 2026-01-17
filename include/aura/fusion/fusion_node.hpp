/**
 * @file fusion_node.hpp
 * @brief Fusion pipeline node base classes
 *
 * Provides base classes for creating fusion pipeline nodes
 * that process and combine sensor data.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/logging.hpp>
#include <aura/core/timestamp.hpp>
#include <aura/core/types.hpp>
#include <aura/fusion/fusion_concepts.hpp>
#include <aura/sensors/sensor_base.hpp>

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace aura {

/**
 * @brief Node state enumeration
 */
enum class NodeState : std::uint8_t {
    Uninitialized = 0,
    Initialized = 1,
    Running = 2,
    Paused = 3,
    Stopped = 4,
    Error = 5
};

/**
 * @brief Configuration for fusion nodes
 */
struct FusionNodeConfig {
    std::string name;
    std::size_t input_buffer_size{32};   ///< Maximum inputs to buffer
    std::size_t output_buffer_size{16};  ///< Maximum outputs to buffer
    Float max_latency_ms{50.0};          ///< Maximum processing latency
    bool log_performance{false};         ///< Log performance metrics
};

/**
 * @brief Statistics for fusion nodes
 */
struct FusionNodeStats {
    std::uint64_t inputs_processed{0};
    std::uint64_t outputs_generated{0};
    std::uint64_t errors{0};
    double avg_latency_ms{0.0};
    double max_latency_ms{0.0};
    double min_latency_ms{std::numeric_limits<double>::max()};
};

/**
 * @brief Abstract base class for fusion pipeline nodes
 *
 * Fusion nodes are the building blocks of the processing pipeline.
 * Each node receives input data, processes it, and produces output data.
 */
class FusionNodeBase {
public:
    using InputPtr = std::unique_ptr<SensorDataBase>;
    using OutputPtr = std::unique_ptr<SensorDataBase>;
    using OutputCallback = std::function<void(OutputPtr)>;

    explicit FusionNodeBase(FusionNodeConfig config) : config_(std::move(config)) {}

    virtual ~FusionNodeBase() = default;

    // Non-copyable
    FusionNodeBase(const FusionNodeBase&) = delete;
    FusionNodeBase& operator=(const FusionNodeBase&) = delete;

    // Movable
    FusionNodeBase(FusionNodeBase&&) noexcept = default;
    FusionNodeBase& operator=(FusionNodeBase&&) noexcept = default;

    /// Initialize the node
    [[nodiscard]] virtual bool initialize() = 0;

    /// Start the node
    [[nodiscard]] virtual bool start() = 0;

    /// Stop the node
    virtual void stop() = 0;

    /// Process a single input
    [[nodiscard]] virtual OutputPtr process(InputPtr input) = 0;

    /// Process multiple inputs (for multi-input fusion)
    [[nodiscard]] virtual OutputPtr processMultiple(std::vector<InputPtr>& inputs) {
        // Default implementation: process first input only
        if (!inputs.empty()) {
            return process(std::move(inputs.front()));
        }
        return nullptr;
    }

    /// Submit input data to the node
    virtual void submitInput(InputPtr input) {
        std::lock_guard lock(input_mutex_);
        if (input_buffer_.size() >= config_.input_buffer_size) {
            // Drop oldest input
            input_buffer_.pop_front();
            ++stats_.errors;
        }
        input_buffer_.push_back(std::move(input));
    }

    /// Set the output callback
    void setOutputCallback(OutputCallback callback) { output_callback_ = std::move(callback); }

    /// Get node name
    [[nodiscard]] std::string_view name() const noexcept { return config_.name; }

    /// Get node state
    [[nodiscard]] NodeState state() const noexcept {
        return state_.load(std::memory_order_acquire);
    }

    /// Get configuration
    [[nodiscard]] const FusionNodeConfig& config() const noexcept { return config_; }

    /// Get statistics
    [[nodiscard]] FusionNodeStats stats() const noexcept {
        std::lock_guard lock(stats_mutex_);
        return stats_;
    }

protected:
    /// Set the node state
    void setState(NodeState state) noexcept { state_.store(state, std::memory_order_release); }

    /// Send output through the callback
    void deliverOutput(OutputPtr output) {
        if (output_callback_ && output) {
            output_callback_(std::move(output));
        }
        ++stats_.outputs_generated;
    }

    /// Update latency statistics
    void updateLatency(double latency_ms) {
        std::lock_guard lock(stats_mutex_);
        ++stats_.inputs_processed;

        // Update running average
        double n = static_cast<double>(stats_.inputs_processed);
        stats_.avg_latency_ms = stats_.avg_latency_ms * ((n - 1) / n) + latency_ms / n;

        stats_.max_latency_ms = std::max(stats_.max_latency_ms, latency_ms);
        stats_.min_latency_ms = std::min(stats_.min_latency_ms, latency_ms);
    }

    /// Get next input from buffer
    [[nodiscard]] InputPtr popInput() {
        std::lock_guard lock(input_mutex_);
        if (input_buffer_.empty()) {
            return nullptr;
        }
        auto input = std::move(input_buffer_.front());
        input_buffer_.pop_front();
        return input;
    }

    /// Check if inputs are available
    [[nodiscard]] bool hasInput() const {
        std::lock_guard lock(input_mutex_);
        return !input_buffer_.empty();
    }

    FusionNodeConfig config_;
    std::atomic<NodeState> state_{NodeState::Uninitialized};

    mutable std::mutex input_mutex_;
    std::deque<InputPtr> input_buffer_;

    OutputCallback output_callback_;

    mutable std::mutex stats_mutex_;
    FusionNodeStats stats_;
};

/**
 * @brief Template-based fusion node for type-safe processing
 *
 * @tparam InputT Input data type
 * @tparam OutputT Output data type
 */
template <typename InputT, typename OutputT>
class TypedFusionNode : public FusionNodeBase {
public:
    using InputType = InputT;
    using OutputType = OutputT;

    explicit TypedFusionNode(FusionNodeConfig config) : FusionNodeBase(std::move(config)) {}

    /// Type-safe processing method
    [[nodiscard]] virtual std::unique_ptr<OutputT> processTyped(std::unique_ptr<InputT> input) = 0;

    /// Implementation of base process method
    [[nodiscard]] OutputPtr process(InputPtr input) override {
        // Try to cast input to expected type
        auto* typed_ptr = dynamic_cast<SensorData<InputT>*>(input.get());
        if (!typed_ptr) {
            AURA_LOG_WARN("TypedFusionNode [{}]: Received unexpected input type", name());
            return nullptr;
        }

        ScopedTimer timer;

        // Extract the data and process
        auto typed_input = std::make_unique<InputT>(typed_ptr->extractData());
        auto result = processTyped(std::move(typed_input));

        updateLatency(static_cast<double>(timer.elapsedMs()));

        if (result) {
            return std::make_unique<SensorData<OutputT>>(std::move(*result), Timestamp::now(), 0, 0,
                                                         std::string(name()));
        }
        return nullptr;
    }
};

/**
 * @brief Pass-through node for debugging
 */
class PassThroughNode : public FusionNodeBase {
public:
    explicit PassThroughNode(FusionNodeConfig config) : FusionNodeBase(std::move(config)) {}

    [[nodiscard]] bool initialize() override {
        setState(NodeState::Initialized);
        return true;
    }

    [[nodiscard]] bool start() override {
        setState(NodeState::Running);
        return true;
    }

    void stop() override { setState(NodeState::Stopped); }

    [[nodiscard]] OutputPtr process(InputPtr input) override {
        // Just pass through
        return input;
    }
};

/**
 * @brief Multi-input fusion node base class
 *
 * Collects inputs from multiple sensors and fuses them when
 * sufficient data is available.
 */
class MultiInputFusionNode : public FusionNodeBase {
public:
    explicit MultiInputFusionNode(FusionNodeConfig config, std::size_t num_inputs)
        : FusionNodeBase(std::move(config)), num_inputs_(num_inputs), input_buffers_(num_inputs) {}

    /// Submit input from a specific sensor
    void submitInput(std::size_t sensor_idx, InputPtr input) {
        if (sensor_idx >= num_inputs_) {
            return;
        }

        std::lock_guard lock(multi_input_mutex_);
        auto& buffer = input_buffers_[sensor_idx];
        if (buffer.size() >= config_.input_buffer_size) {
            buffer.pop_front();
        }
        buffer.push_back(std::move(input));
    }

    /// Check if we have inputs from all sensors
    [[nodiscard]] bool hasAllInputs() const {
        std::lock_guard lock(multi_input_mutex_);
        for (const auto& buffer : input_buffers_) {
            if (buffer.empty()) {
                return false;
            }
        }
        return true;
    }

    /// Get the number of expected inputs
    [[nodiscard]] std::size_t numInputs() const noexcept { return num_inputs_; }

protected:
    /// Get inputs from all sensors (one from each)
    [[nodiscard]] std::vector<InputPtr> collectInputs() {
        std::lock_guard lock(multi_input_mutex_);
        std::vector<InputPtr> inputs;
        inputs.reserve(num_inputs_);

        for (auto& buffer : input_buffers_) {
            if (!buffer.empty()) {
                inputs.push_back(std::move(buffer.front()));
                buffer.pop_front();
            }
        }
        return inputs;
    }

    /// Find temporally aligned inputs
    [[nodiscard]] std::vector<InputPtr> findAlignedInputs(Timestamp target,
                                                          Timestamp::Duration tolerance) {
        std::lock_guard lock(multi_input_mutex_);
        std::vector<InputPtr> inputs;
        inputs.reserve(num_inputs_);

        for (auto& buffer : input_buffers_) {
            InputPtr best_match;

            for (auto it = buffer.begin(); it != buffer.end();) {
                if ((*it)->timestamp().isWithin(target, tolerance)) {
                    best_match = std::move(*it);
                    it = buffer.erase(it);
                    break;
                } else if ((*it)->timestamp() < target - Timestamp(tolerance.count())) {
                    // Too old, discard
                    it = buffer.erase(it);
                } else {
                    ++it;
                }
            }

            if (best_match) {
                inputs.push_back(std::move(best_match));
            }
        }
        return inputs;
    }

private:
    std::size_t num_inputs_;
    mutable std::mutex multi_input_mutex_;
    std::vector<std::deque<InputPtr>> input_buffers_;
};

}  // namespace aura
