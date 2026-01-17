/**
 * @file timestamp.hpp
 * @brief High-precision timestamp utilities for sensor data synchronization
 *
 * This file provides timestamp types and utilities for handling temporal
 * aspects of sensor data, including synchronization and temporal alignment.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <chrono>
#include <compare>
#include <cstdint>
#include <limits>

namespace aura {

/**
 * @brief High-precision timestamp for sensor data
 *
 * Uses nanosecond precision internally but provides convenient
 * conversions to various time units. Supports comparison and
 * arithmetic operations.
 */
class Timestamp {
public:
    /// Underlying duration type (nanoseconds)
    using Duration = std::chrono::nanoseconds;

    /// Clock type used for wall-clock time
    using Clock = std::chrono::high_resolution_clock;

    /// Default constructor (zero timestamp)
    constexpr Timestamp() noexcept = default;

    /// Construct from nanoseconds
    explicit constexpr Timestamp(std::int64_t nanoseconds) noexcept : nanoseconds_(nanoseconds) {}

    /// Construct from duration
    explicit constexpr Timestamp(Duration duration) noexcept : nanoseconds_(duration.count()) {}

    /// Get current wall-clock time
    [[nodiscard]] static Timestamp now() noexcept {
        return Timestamp(
            std::chrono::duration_cast<Duration>(Clock::now().time_since_epoch()).count());
    }

    /// Create from seconds (double precision)
    [[nodiscard]] static constexpr Timestamp fromSeconds(double seconds) noexcept {
        return Timestamp(static_cast<std::int64_t>(seconds * 1e9));
    }

    /// Create from milliseconds
    [[nodiscard]] static constexpr Timestamp fromMilliseconds(std::int64_t ms) noexcept {
        return Timestamp(ms * 1'000'000);
    }

    /// Create from microseconds
    [[nodiscard]] static constexpr Timestamp fromMicroseconds(std::int64_t us) noexcept {
        return Timestamp(us * 1'000);
    }

    /// Create from nanoseconds
    [[nodiscard]] static constexpr Timestamp fromNanoseconds(std::int64_t ns) noexcept {
        return Timestamp(ns);
    }

    /// Get nanoseconds as integer
    [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept { return nanoseconds_; }

    /// Get microseconds as integer (truncated)
    [[nodiscard]] constexpr std::int64_t microseconds() const noexcept {
        return nanoseconds_ / 1'000;
    }

    /// Get milliseconds as integer (truncated)
    [[nodiscard]] constexpr std::int64_t milliseconds() const noexcept {
        return nanoseconds_ / 1'000'000;
    }

    /// Get seconds as double
    [[nodiscard]] constexpr double seconds() const noexcept {
        return static_cast<double>(nanoseconds_) / 1e9;
    }

    /// Get as std::chrono::duration
    [[nodiscard]] constexpr Duration toDuration() const noexcept { return Duration(nanoseconds_); }

    /// Check if timestamp is zero/unset
    [[nodiscard]] constexpr bool isZero() const noexcept { return nanoseconds_ == 0; }

    /// Check if timestamp is valid (non-negative)
    [[nodiscard]] constexpr bool isValid() const noexcept { return nanoseconds_ >= 0; }

    /// Maximum representable timestamp
    [[nodiscard]] static constexpr Timestamp max() noexcept {
        return Timestamp(std::numeric_limits<std::int64_t>::max());
    }

    /// Minimum representable timestamp
    [[nodiscard]] static constexpr Timestamp min() noexcept {
        return Timestamp(std::numeric_limits<std::int64_t>::min());
    }

    // Arithmetic operators

    /// Add two timestamps
    [[nodiscard]] constexpr Timestamp operator+(Timestamp other) const noexcept {
        return Timestamp(nanoseconds_ + other.nanoseconds_);
    }

    /// Subtract two timestamps
    [[nodiscard]] constexpr Timestamp operator-(Timestamp other) const noexcept {
        return Timestamp(nanoseconds_ - other.nanoseconds_);
    }

    /// Add duration to timestamp
    [[nodiscard]] constexpr Timestamp operator+(Duration duration) const noexcept {
        return Timestamp(nanoseconds_ + duration.count());
    }

    /// Subtract duration from timestamp
    [[nodiscard]] constexpr Timestamp operator-(Duration duration) const noexcept {
        return Timestamp(nanoseconds_ - duration.count());
    }

    /// Compound addition
    constexpr Timestamp& operator+=(Timestamp other) noexcept {
        nanoseconds_ += other.nanoseconds_;
        return *this;
    }

    /// Compound subtraction
    constexpr Timestamp& operator-=(Timestamp other) noexcept {
        nanoseconds_ -= other.nanoseconds_;
        return *this;
    }

    // Comparison operators (using C++20 spaceship operator)
    [[nodiscard]] constexpr auto operator<=>(const Timestamp& other) const noexcept = default;

    /// Compute absolute difference between timestamps
    [[nodiscard]] constexpr Timestamp absDiff(Timestamp other) const noexcept {
        auto diff = nanoseconds_ - other.nanoseconds_;
        return Timestamp(diff >= 0 ? diff : -diff);
    }

    /// Check if this timestamp is within a tolerance of another
    [[nodiscard]] constexpr bool isWithin(Timestamp other, Duration tolerance) const noexcept {
        return absDiff(other).nanoseconds_ <= tolerance.count();
    }

private:
    std::int64_t nanoseconds_{0};
};

/**
 * @brief Time interval/duration wrapper for clearer semantics
 */
class TimeInterval {
public:
    constexpr TimeInterval() noexcept = default;

    explicit constexpr TimeInterval(Timestamp::Duration duration) noexcept : duration_(duration) {}

    [[nodiscard]] static constexpr TimeInterval fromSeconds(double seconds) noexcept {
        return TimeInterval(std::chrono::duration_cast<Timestamp::Duration>(
            std::chrono::duration<double>(seconds)));
    }

    [[nodiscard]] static constexpr TimeInterval fromMilliseconds(std::int64_t ms) noexcept {
        return TimeInterval(std::chrono::milliseconds(ms));
    }

    [[nodiscard]] constexpr Timestamp::Duration duration() const noexcept { return duration_; }

    [[nodiscard]] constexpr double seconds() const noexcept {
        return std::chrono::duration<double>(duration_).count();
    }

    [[nodiscard]] constexpr std::int64_t milliseconds() const noexcept {
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration_).count();
    }

private:
    Timestamp::Duration duration_{0};
};

/**
 * @brief Scoped timer for measuring elapsed time
 *
 * Usage:
 * @code
 * {
 *     ScopedTimer timer;
 *     // ... do work ...
 *     auto elapsed_ms = timer.elapsedMs();
 * }
 * @endcode
 */
class ScopedTimer {
public:
    ScopedTimer() noexcept : start_(Timestamp::now()) {}

    /// Reset the timer
    void reset() noexcept { start_ = Timestamp::now(); }

    /// Get elapsed time as Timestamp
    [[nodiscard]] Timestamp elapsed() const noexcept { return Timestamp::now() - start_; }

    /// Get elapsed time in seconds
    [[nodiscard]] double elapsedSeconds() const noexcept { return elapsed().seconds(); }

    /// Get elapsed time in milliseconds
    [[nodiscard]] std::int64_t elapsedMs() const noexcept { return elapsed().milliseconds(); }

    /// Get elapsed time in microseconds
    [[nodiscard]] std::int64_t elapsedUs() const noexcept { return elapsed().microseconds(); }

    /// Get elapsed time in nanoseconds
    [[nodiscard]] std::int64_t elapsedNs() const noexcept { return elapsed().nanoseconds(); }

private:
    Timestamp start_;
};

}  // namespace aura
