/**
 * @file aura.hpp
 * @brief Main header file for the AURA sensor fusion library
 *
 * AURA: Autonomous Unified Real-time Architecture
 * A high-performance, real-time multi-sensor fusion system for autonomous vehicles.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

// Version information
#define AURA_VERSION_MAJOR 1
#define AURA_VERSION_MINOR 0
#define AURA_VERSION_PATCH 0
#define AURA_VERSION_STRING "1.0.0"

// Core includes
#include <aura/core/logging.hpp>
#include <aura/core/memory_pool.hpp>
#include <aura/core/timestamp.hpp>
#include <aura/core/types.hpp>

// Concurrency includes
#include <aura/concurrency/lock_free_ring_buffer.hpp>
#include <aura/concurrency/thread_pool.hpp>

// Sensor includes
#include <aura/sensors/sensor_base.hpp>
#include <aura/sensors/sensor_data.hpp>
#include <aura/sensors/sensor_traits.hpp>

// Fusion includes
#include <aura/fusion/fusion_concepts.hpp>
#include <aura/fusion/fusion_node.hpp>

// Utils includes
#include <aura/utils/profiler.hpp>

namespace aura {

/**
 * @brief Get the library version string
 * @return Version string in format "major.minor.patch"
 */
[[nodiscard]] constexpr const char* version() noexcept {
    return AURA_VERSION_STRING;
}

/**
 * @brief Get the major version number
 * @return Major version number
 */
[[nodiscard]] constexpr int versionMajor() noexcept {
    return AURA_VERSION_MAJOR;
}

/**
 * @brief Get the minor version number
 * @return Minor version number
 */
[[nodiscard]] constexpr int versionMinor() noexcept {
    return AURA_VERSION_MINOR;
}

/**
 * @brief Get the patch version number
 * @return Patch version number
 */
[[nodiscard]] constexpr int versionPatch() noexcept {
    return AURA_VERSION_PATCH;
}

}  // namespace aura
