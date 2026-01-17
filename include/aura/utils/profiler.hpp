/**
 * @file profiler.hpp
 * @brief Performance profiling utilities
 *
 * Provides tools for measuring and analyzing performance:
 * - Scoped timers for measuring execution time
 * - Statistical aggregation of measurements
 * - Thread-safe profiling with named regions
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/timestamp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace aura {

/**
 * @brief Statistics for a profiled region
 */
struct ProfileStats {
    std::uint64_t count{0};
    double total_ms{0.0};
    double min_ms{std::numeric_limits<double>::max()};
    double max_ms{0.0};
    double mean_ms{0.0};
    double variance_ms{0.0};

    /// Compute standard deviation
    [[nodiscard]] double stddev_ms() const noexcept { return std::sqrt(variance_ms); }

    /// Compute average
    [[nodiscard]] double avg_ms() const noexcept {
        return count > 0 ? total_ms / static_cast<double>(count) : 0.0;
    }
};

/**
 * @brief Thread-safe profiler for measuring execution times
 *
 * Usage:
 * @code
 * // Profile a code block
 * {
 *     auto scope = Profiler::instance().scope("my_function");
 *     // ... code to profile ...
 * }
 *
 * // Get statistics
 * auto stats = Profiler::instance().stats("my_function");
 * @endcode
 */
class Profiler {
public:
    /**
     * @brief Scoped profiling helper
     *
     * Measures time from construction to destruction.
     */
    class ScopedProfile {
    public:
        ScopedProfile(Profiler& profiler, std::string_view name)
            : profiler_(profiler), name_(name), start_(Timestamp::now()) {}

        ~ScopedProfile() {
            auto elapsed = Timestamp::now() - start_;
            profiler_.record(name_, static_cast<double>(elapsed.nanoseconds()) / 1e6);
        }

        // Non-copyable, non-movable
        ScopedProfile(const ScopedProfile&) = delete;
        ScopedProfile& operator=(const ScopedProfile&) = delete;
        ScopedProfile(ScopedProfile&&) = delete;
        ScopedProfile& operator=(ScopedProfile&&) = delete;

    private:
        Profiler& profiler_;
        std::string_view name_;
        Timestamp start_;
    };

    /// Get the global profiler instance
    static Profiler& instance() {
        static Profiler profiler;
        return profiler;
    }

    /**
     * @brief Create a scoped profile for a named region
     *
     * @param name Region name
     * @return ScopedProfile that records time on destruction
     */
    [[nodiscard]] ScopedProfile scope(std::string_view name) { return ScopedProfile(*this, name); }

    /**
     * @brief Record a timing measurement
     *
     * @param name Region name
     * @param duration_ms Duration in milliseconds
     */
    void record(std::string_view name, double duration_ms) {
        std::lock_guard lock(mutex_);

        auto& data = regions_[std::string(name)];
        ++data.count;
        data.total_ms += duration_ms;
        data.min_ms = std::min(data.min_ms, duration_ms);
        data.max_ms = std::max(data.max_ms, duration_ms);

        // Welford's online algorithm for variance
        double n = static_cast<double>(data.count);
        double delta = duration_ms - data.mean_ms;
        data.mean_ms += delta / n;
        double delta2 = duration_ms - data.mean_ms;
        data.variance_ms += delta * delta2;
    }

    /**
     * @brief Get statistics for a region
     *
     * @param name Region name
     * @return ProfileStats for the region
     */
    [[nodiscard]] ProfileStats stats(std::string_view name) const {
        std::lock_guard lock(mutex_);

        auto it = regions_.find(std::string(name));
        if (it == regions_.end()) {
            return {};
        }

        ProfileStats result = it->second;
        if (result.count > 1) {
            result.variance_ms /= static_cast<double>(result.count - 1);
        }
        return result;
    }

    /**
     * @brief Get statistics for all regions
     *
     * @return Map of region names to ProfileStats
     */
    [[nodiscard]] std::map<std::string, ProfileStats> allStats() const {
        std::lock_guard lock(mutex_);

        std::map<std::string, ProfileStats> result;
        for (const auto& [name, data] : regions_) {
            ProfileStats stats = data;
            if (stats.count > 1) {
                stats.variance_ms /= static_cast<double>(stats.count - 1);
            }
            result[name] = stats;
        }
        return result;
    }

    /**
     * @brief Reset all profiling data
     */
    void reset() {
        std::lock_guard lock(mutex_);
        regions_.clear();
    }

    /**
     * @brief Reset data for a specific region
     */
    void reset(std::string_view name) {
        std::lock_guard lock(mutex_);
        regions_.erase(std::string(name));
    }

    /**
     * @brief Check if a region has been profiled
     */
    [[nodiscard]] bool hasRegion(std::string_view name) const {
        std::lock_guard lock(mutex_);
        return regions_.contains(std::string(name));
    }

    /**
     * @brief Get list of all profiled region names
     */
    [[nodiscard]] std::vector<std::string> regionNames() const {
        std::lock_guard lock(mutex_);

        std::vector<std::string> names;
        names.reserve(regions_.size());
        for (const auto& [name, _] : regions_) {
            names.push_back(name);
        }
        return names;
    }

    /**
     * @brief Generate a report string
     */
    [[nodiscard]] std::string report() const {
        auto all = allStats();

        std::string result = "Profiler Report\n";
        result += "===============\n\n";
        result += "| Region | Count | Mean (ms) | Min (ms) | Max (ms) | StdDev (ms) |\n";
        result += "|--------|-------|-----------|----------|----------|-------------|\n";

        for (const auto& [name, stats] : all) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "| %-30s | %7lu | %9.3f | %8.3f | %8.3f | %11.3f |\n",
                          name.c_str(), static_cast<unsigned long>(stats.count), stats.mean_ms,
                          stats.min_ms, stats.max_ms, stats.stddev_ms());
            result += buf;
        }

        return result;
    }

private:
    Profiler() = default;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, ProfileStats> regions_;
};

/**
 * @brief Helper macro for scoped profiling
 */
#define AURA_PROFILE_SCOPE(name) \
    auto AURA_PROFILE_##__LINE__ = ::aura::Profiler::instance().scope(name)

/**
 * @brief Helper macro for function profiling
 */
#define AURA_PROFILE_FUNCTION() AURA_PROFILE_SCOPE(__func__)

/**
 * @brief Frame rate tracker
 *
 * Tracks and computes frame rate statistics.
 */
class FrameRateTracker {
public:
    /**
     * @brief Construct with window size for averaging
     *
     * @param window_size Number of frames for moving average
     */
    explicit FrameRateTracker(std::size_t window_size = 60) : window_size_(window_size) {
        frame_times_.reserve(window_size);
    }

    /**
     * @brief Record a new frame
     */
    void tick() {
        auto now = Timestamp::now();

        if (!last_frame_time_.isZero()) {
            auto delta = (now - last_frame_time_).seconds();

            std::lock_guard lock(mutex_);
            if (frame_times_.size() >= window_size_) {
                frame_times_.erase(frame_times_.begin());
            }
            frame_times_.push_back(delta);
        }

        last_frame_time_ = now;
    }

    /**
     * @brief Get current frame rate (FPS)
     */
    [[nodiscard]] double fps() const {
        std::lock_guard lock(mutex_);

        if (frame_times_.empty()) {
            return 0.0;
        }

        double sum = 0.0;
        for (double t : frame_times_) {
            sum += t;
        }

        double avg_delta = sum / static_cast<double>(frame_times_.size());
        return avg_delta > 0.0 ? 1.0 / avg_delta : 0.0;
    }

    /**
     * @brief Get average frame time in milliseconds
     */
    [[nodiscard]] double avgFrameTimeMs() const {
        std::lock_guard lock(mutex_);

        if (frame_times_.empty()) {
            return 0.0;
        }

        double sum = 0.0;
        for (double t : frame_times_) {
            sum += t;
        }

        return (sum / static_cast<double>(frame_times_.size())) * 1000.0;
    }

    /**
     * @brief Reset the tracker
     */
    void reset() {
        std::lock_guard lock(mutex_);
        frame_times_.clear();
        last_frame_time_ = Timestamp();
    }

private:
    std::size_t window_size_;
    mutable std::mutex mutex_;
    std::vector<double> frame_times_;
    Timestamp last_frame_time_;
};

}  // namespace aura
