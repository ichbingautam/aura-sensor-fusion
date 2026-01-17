/**
 * @file logging.hpp
 * @brief Lightweight logging system for AURA
 *
 * Provides a simple but flexible logging interface with:
 * - Multiple log levels (TRACE, DEBUG, INFO, WARN, ERROR, FATAL)
 * - Compile-time log level filtering
 * - Thread-safe output
 * - Customizable output sinks
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/timestamp.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <format>
#include <functional>
#include <mutex>
#include <source_location>
#include <string>
#include <string_view>
#include <thread>

namespace aura {

/**
 * @brief Log severity levels
 */
enum class LogLevel : int {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
    Fatal = 5,
    Off = 6
};

/**
 * @brief Convert log level to string
 */
[[nodiscard]] constexpr std::string_view toString(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Trace:
            return "TRACE";
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warn:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Fatal:
            return "FATAL";
        case LogLevel::Off:
            return "OFF";
    }
    return "UNKNOWN";
}

/**
 * @brief Get ANSI color code for log level
 */
[[nodiscard]] constexpr std::string_view getColorCode(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Trace:
            return "\033[90m";  // Gray
        case LogLevel::Debug:
            return "\033[36m";  // Cyan
        case LogLevel::Info:
            return "\033[32m";  // Green
        case LogLevel::Warn:
            return "\033[33m";  // Yellow
        case LogLevel::Error:
            return "\033[31m";  // Red
        case LogLevel::Fatal:
            return "\033[35;1m";  // Bold Magenta
        case LogLevel::Off:
            return "";
    }
    return "";
}

/// ANSI reset code
inline constexpr std::string_view kColorReset = "\033[0m";

/**
 * @brief Log message structure
 */
struct LogMessage {
    LogLevel level;
    std::string message;
    Timestamp timestamp;
    std::source_location location;
    std::thread::id thread_id;
};

/**
 * @brief Logging sink interface
 */
class LogSink {
public:
    virtual ~LogSink() = default;
    virtual void write(const LogMessage& msg) = 0;
    virtual void flush() = 0;
};

/**
 * @brief Console log sink with color support
 */
class ConsoleSink : public LogSink {
public:
    explicit ConsoleSink(bool use_colors = true, FILE* output = stderr)
        : use_colors_(use_colors), output_(output) {}

    void write(const LogMessage& msg) override {
        std::lock_guard lock(mutex_);

        if (use_colors_) {
            std::fprintf(output_, "%.*s[%.*s]%.*s ",
                         static_cast<int>(getColorCode(msg.level).size()),
                         getColorCode(msg.level).data(),
                         static_cast<int>(toString(msg.level).size()), toString(msg.level).data(),
                         static_cast<int>(kColorReset.size()), kColorReset.data());
        } else {
            std::fprintf(output_, "[%.*s] ", static_cast<int>(toString(msg.level).size()),
                         toString(msg.level).data());
        }

        // Print timestamp (simplified format)
        auto time_ms = msg.timestamp.milliseconds();
        std::fprintf(output_, "[%lld.%03lld] ", time_ms / 1000, time_ms % 1000);

        // Print message
        std::fprintf(output_, "%s", msg.message.c_str());

        // Print source location for debug and above
        if (msg.level >= LogLevel::Debug) {
            std::fprintf(output_, " (%s:%d)", msg.location.file_name(), msg.location.line());
        }

        std::fprintf(output_, "\n");
    }

    void flush() override {
        std::lock_guard lock(mutex_);
        std::fflush(output_);
    }

private:
    bool use_colors_;
    FILE* output_;
    std::mutex mutex_;
};

/**
 * @brief Global logger singleton
 */
class Logger {
public:
    /// Get the global logger instance
    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    /// Set the minimum log level
    void setLevel(LogLevel level) noexcept { min_level_.store(level, std::memory_order_relaxed); }

    /// Get the current log level
    [[nodiscard]] LogLevel level() const noexcept {
        return min_level_.load(std::memory_order_relaxed);
    }

    /// Check if a log level is enabled
    [[nodiscard]] bool isEnabled(LogLevel level) const noexcept {
        return level >= min_level_.load(std::memory_order_relaxed);
    }

    /// Add a log sink
    void addSink(std::shared_ptr<LogSink> sink) {
        std::lock_guard lock(sinks_mutex_);
        sinks_.push_back(std::move(sink));
    }

    /// Clear all sinks
    void clearSinks() {
        std::lock_guard lock(sinks_mutex_);
        sinks_.clear();
    }

    /// Log a message
    template <typename... Args>
    void log(LogLevel level, std::source_location loc, std::format_string<Args...> fmt,
             Args&&... args) {
        if (!isEnabled(level)) {
            return;
        }

        LogMessage msg{.level = level,
                       .message = std::format(fmt, std::forward<Args>(args)...),
                       .timestamp = Timestamp::now(),
                       .location = loc,
                       .thread_id = std::this_thread::get_id()};

        std::lock_guard lock(sinks_mutex_);
        for (auto& sink : sinks_) {
            sink->write(msg);
        }
    }

    /// Flush all sinks
    void flush() {
        std::lock_guard lock(sinks_mutex_);
        for (auto& sink : sinks_) {
            sink->flush();
        }
    }

private:
    Logger() {
        // Add default console sink
        sinks_.push_back(std::make_shared<ConsoleSink>());
        min_level_.store(LogLevel::Info, std::memory_order_relaxed);
    }

    std::atomic<LogLevel> min_level_{LogLevel::Info};
    std::mutex sinks_mutex_;
    std::vector<std::shared_ptr<LogSink>> sinks_;
};

// ============================================================================
// Logging Macros
// ============================================================================

/// Log at TRACE level
#define AURA_LOG_TRACE(...)                                                               \
    do {                                                                                  \
        if (::aura::Logger::instance().isEnabled(::aura::LogLevel::Trace)) {              \
            ::aura::Logger::instance().log(::aura::LogLevel::Trace,                       \
                                           std::source_location::current(), __VA_ARGS__); \
        }                                                                                 \
    } while (false)

/// Log at DEBUG level
#define AURA_LOG_DEBUG(...)                                                               \
    do {                                                                                  \
        if (::aura::Logger::instance().isEnabled(::aura::LogLevel::Debug)) {              \
            ::aura::Logger::instance().log(::aura::LogLevel::Debug,                       \
                                           std::source_location::current(), __VA_ARGS__); \
        }                                                                                 \
    } while (false)

/// Log at INFO level
#define AURA_LOG_INFO(...)                                                                \
    do {                                                                                  \
        if (::aura::Logger::instance().isEnabled(::aura::LogLevel::Info)) {               \
            ::aura::Logger::instance().log(::aura::LogLevel::Info,                        \
                                           std::source_location::current(), __VA_ARGS__); \
        }                                                                                 \
    } while (false)

/// Log at WARN level
#define AURA_LOG_WARN(...)                                                                \
    do {                                                                                  \
        if (::aura::Logger::instance().isEnabled(::aura::LogLevel::Warn)) {               \
            ::aura::Logger::instance().log(::aura::LogLevel::Warn,                        \
                                           std::source_location::current(), __VA_ARGS__); \
        }                                                                                 \
    } while (false)

/// Log at ERROR level
#define AURA_LOG_ERROR(...)                                                               \
    do {                                                                                  \
        if (::aura::Logger::instance().isEnabled(::aura::LogLevel::Error)) {              \
            ::aura::Logger::instance().log(::aura::LogLevel::Error,                       \
                                           std::source_location::current(), __VA_ARGS__); \
        }                                                                                 \
    } while (false)

/// Log at FATAL level
#define AURA_LOG_FATAL(...)                                                               \
    do {                                                                                  \
        if (::aura::Logger::instance().isEnabled(::aura::LogLevel::Fatal)) {              \
            ::aura::Logger::instance().log(::aura::LogLevel::Fatal,                       \
                                           std::source_location::current(), __VA_ARGS__); \
        }                                                                                 \
    } while (false)

}  // namespace aura
