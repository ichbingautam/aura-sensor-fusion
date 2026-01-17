/**
 * @file sensor_traits.hpp
 * @brief Type traits for sensor data handling
 *
 * Provides compile-time type traits for sensor data types,
 * enabling type-safe template metaprogramming.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/types.hpp>
#include <aura/sensors/sensor_base.hpp>

#include <type_traits>

namespace aura {

/**
 * @brief Primary template for sensor traits
 *
 * Specialize this template for custom sensor types.
 */
template <typename SensorT>
struct SensorTraits {
    /// The raw data type produced by the sensor
    using raw_type = void;

    /// The normalized data type used in the pipeline
    using normalized_type = void;

    /// Sensor type enumeration value
    static constexpr SensorType sensor_type = SensorType::Unknown;

    /// Whether the sensor produces continuous data (vs discrete events)
    static constexpr bool is_continuous = false;

    /// Expected update rate in Hz (0 = unknown)
    static constexpr double nominal_rate_hz = 0.0;
};

// Forward declarations for sensor types
class LidarSensor;
class CameraSensor;
class RadarSensor;
class GpsSensor;
class ImuSensor;

/**
 * @brief Traits specialization for LiDAR sensors
 */
template <>
struct SensorTraits<LidarSensor> {
    using raw_type = std::vector<LidarPoint>;
    using normalized_type = std::vector<Point3D>;
    static constexpr SensorType sensor_type = SensorType::Lidar;
    static constexpr bool is_continuous = true;
    static constexpr double nominal_rate_hz = 10.0;
};

/**
 * @brief Traits specialization for cameras
 */
template <>
struct SensorTraits<CameraSensor> {
    // For cameras, raw type would typically be image data
    using raw_type = std::vector<Byte>;
    using normalized_type = std::vector<DetectedObject>;
    static constexpr SensorType sensor_type = SensorType::Camera;
    static constexpr bool is_continuous = true;
    static constexpr double nominal_rate_hz = 30.0;
};

/**
 * @brief Traits specialization for radar
 */
template <>
struct SensorTraits<RadarSensor> {
    using raw_type = std::vector<RadarPoint>;
    using normalized_type = std::vector<DetectedObject>;
    static constexpr SensorType sensor_type = SensorType::Radar;
    static constexpr bool is_continuous = true;
    static constexpr double nominal_rate_hz = 20.0;
};

/**
 * @brief Traits specialization for GPS
 */
template <>
struct SensorTraits<GpsSensor> {
    using raw_type = GpsPosition;
    using normalized_type = Pose3D;
    static constexpr SensorType sensor_type = SensorType::Gps;
    static constexpr bool is_continuous = false;
    static constexpr double nominal_rate_hz = 10.0;
};

/**
 * @brief Traits specialization for IMU
 */
template <>
struct SensorTraits<ImuSensor> {
    using raw_type = ImuData;
    using normalized_type = ImuData;
    static constexpr SensorType sensor_type = SensorType::Imu;
    static constexpr bool is_continuous = true;
    static constexpr double nominal_rate_hz = 100.0;
};

// ============================================================================
// Type Trait Concepts
// ============================================================================

/**
 * @brief Concept for valid sensor types
 */
template <typename T>
concept ValidSensor = requires {
    typename SensorTraits<T>::raw_type;
    typename SensorTraits<T>::normalized_type;
    { SensorTraits<T>::sensor_type } -> std::convertible_to<SensorType>;
};

/**
 * @brief Concept for continuous sensors
 */
template <typename T>
concept ContinuousSensor = ValidSensor<T> && SensorTraits<T>::is_continuous;

/**
 * @brief Concept for sensors that produce point cloud data
 */
template <typename T>
concept PointCloudSensor = ValidSensor<T> && (SensorTraits<T>::sensor_type == SensorType::Lidar ||
                                              SensorTraits<T>::sensor_type == SensorType::Radar);

/**
 * @brief Concept for sensors that produce position data
 */
template <typename T>
concept PositionSensor = ValidSensor<T> && (SensorTraits<T>::sensor_type == SensorType::Gps);

/**
 * @brief Concept for sensors that produce motion data
 */
template <typename T>
concept MotionSensor = ValidSensor<T> && (SensorTraits<T>::sensor_type == SensorType::Imu);

// ============================================================================
// Sensor Data Type Helpers
// ============================================================================

/**
 * @brief Get the raw data type for a sensor
 */
template <typename SensorT>
using RawDataType = typename SensorTraits<SensorT>::raw_type;

/**
 * @brief Get the normalized data type for a sensor
 */
template <typename SensorT>
using NormalizedDataType = typename SensorTraits<SensorT>::normalized_type;

/**
 * @brief Check if two sensor types are compatible for fusion
 */
template <typename Sensor1, typename Sensor2>
struct AreFusionCompatible : std::true_type {};

/**
 * @brief Check if a sensor type produces spatial data
 */
template <typename SensorT>
struct ProducesSpatialData
    : std::bool_constant<SensorTraits<SensorT>::sensor_type == SensorType::Lidar ||
                         SensorTraits<SensorT>::sensor_type == SensorType::Radar ||
                         SensorTraits<SensorT>::sensor_type == SensorType::Camera> {};

}  // namespace aura
