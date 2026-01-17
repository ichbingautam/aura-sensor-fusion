/**
 * @file types.hpp
 * @brief Core type definitions for the AURA sensor fusion system
 *
 * This file contains fundamental type aliases and structures used throughout
 * the AURA library. These types provide a consistent interface for sensor data,
 * spatial coordinates, and mathematical operations.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#ifdef AURA_HAS_EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif

namespace aura {

// ============================================================================
// Fundamental Type Aliases
// ============================================================================

/// Unsigned byte type
using Byte = std::uint8_t;

/// Floating-point type used throughout the library
using Float = double;

/// Single-precision floating-point type for memory-efficient operations
using Float32 = float;

/// 64-bit floating-point type for high-precision calculations
using Float64 = double;

/// Size type for indexing
using Size = std::size_t;

/// Index type (signed for potential negative indexing)
using Index = std::ptrdiff_t;

// ============================================================================
// Sensor Identifiers
// ============================================================================

/// Unique identifier for sensors
using SensorId = std::uint32_t;

/// Sensor sequence number for ordering
using SequenceNumber = std::uint64_t;

// ============================================================================
// 3D Geometry Types
// ============================================================================

/**
 * @brief 3D point structure for spatial coordinates
 *
 * Represents a point in 3D Cartesian space. Used for LiDAR points,
 * object positions, and spatial calculations.
 */
struct Point3D {
    Float x{0.0};  ///< X coordinate (forward in vehicle frame)
    Float y{0.0};  ///< Y coordinate (left in vehicle frame)
    Float z{0.0};  ///< Z coordinate (up in vehicle frame)

    /// Default constructor
    constexpr Point3D() noexcept = default;

    /// Construct from coordinates
    constexpr Point3D(Float x, Float y, Float z) noexcept : x(x), y(y), z(z) {}

    /// Compute Euclidean distance to another point
    [[nodiscard]] constexpr Float distanceTo(const Point3D& other) const noexcept {
        const Float dx = x - other.x;
        const Float dy = y - other.y;
        const Float dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    /// Compute squared distance (avoids sqrt for comparisons)
    [[nodiscard]] constexpr Float distanceSquaredTo(const Point3D& other) const noexcept {
        const Float dx = x - other.x;
        const Float dy = y - other.y;
        const Float dz = z - other.z;
        return dx * dx + dy * dy + dz * dz;
    }

    /// Addition operator
    [[nodiscard]] constexpr Point3D operator+(const Point3D& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    /// Subtraction operator
    [[nodiscard]] constexpr Point3D operator-(const Point3D& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    /// Scalar multiplication
    [[nodiscard]] constexpr Point3D operator*(Float scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    /// Equality comparison
    [[nodiscard]] constexpr bool operator==(const Point3D& other) const noexcept = default;
};

/**
 * @brief 3D vector for directions and velocities
 */
struct Vector3D {
    Float x{0.0};
    Float y{0.0};
    Float z{0.0};

    constexpr Vector3D() noexcept = default;
    constexpr Vector3D(Float x, Float y, Float z) noexcept : x(x), y(y), z(z) {}

    /// Compute vector magnitude
    [[nodiscard]] constexpr Float magnitude() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }

    /// Compute squared magnitude
    [[nodiscard]] constexpr Float magnitudeSquared() const noexcept {
        return x * x + y * y + z * z;
    }

    /// Normalize the vector
    [[nodiscard]] Vector3D normalized() const noexcept {
        const Float mag = magnitude();
        if (mag > 0.0) {
            return {x / mag, y / mag, z / mag};
        }
        return *this;
    }

    /// Dot product
    [[nodiscard]] constexpr Float dot(const Vector3D& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    /// Cross product
    [[nodiscard]] constexpr Vector3D cross(const Vector3D& other) const noexcept {
        return {y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x};
    }

    [[nodiscard]] constexpr Vector3D operator+(const Vector3D& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] constexpr Vector3D operator-(const Vector3D& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] constexpr Vector3D operator*(Float scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }
};

/**
 * @brief Quaternion for 3D rotations
 *
 * Represents orientation in 3D space using quaternion representation.
 * Stored as (w, x, y, z) where w is the scalar component.
 */
struct Quaternion {
    Float w{1.0};  ///< Scalar component
    Float x{0.0};  ///< X component of vector part
    Float y{0.0};  ///< Y component of vector part
    Float z{0.0};  ///< Z component of vector part

    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(Float w, Float x, Float y, Float z) noexcept : w(w), x(x), y(y), z(z) {}

    /// Identity quaternion (no rotation)
    static constexpr Quaternion identity() noexcept { return {1.0, 0.0, 0.0, 0.0}; }

    /// Normalize the quaternion
    [[nodiscard]] Quaternion normalized() const noexcept {
        const Float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0.0) {
            return {w / norm, x / norm, y / norm, z / norm};
        }
        return *this;
    }

    /// Conjugate (inverse for unit quaternions)
    [[nodiscard]] constexpr Quaternion conjugate() const noexcept { return {w, -x, -y, -z}; }

    /// Quaternion multiplication
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& other) const noexcept {
        return {w * other.w - x * other.x - y * other.y - z * other.z,
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w};
    }

    /// Rotate a point by this quaternion
    [[nodiscard]] Point3D rotate(const Point3D& point) const noexcept {
        const Quaternion p{0.0, point.x, point.y, point.z};
        const Quaternion rotated = (*this) * p * conjugate();
        return {rotated.x, rotated.y, rotated.z};
    }
};

/**
 * @brief 6-DOF pose (position + orientation)
 */
struct Pose3D {
    Point3D position;
    Quaternion orientation;

    constexpr Pose3D() noexcept = default;
    constexpr Pose3D(Point3D pos, Quaternion orient) noexcept
        : position(pos), orientation(orient) {}

    /// Identity pose (origin with no rotation)
    static constexpr Pose3D identity() noexcept { return {{0, 0, 0}, Quaternion::identity()}; }

    /// Transform a point from local to world coordinates
    [[nodiscard]] Point3D transformPoint(const Point3D& local) const noexcept {
        return position + orientation.rotate(local);
    }
};

// ============================================================================
// Bounding Box Types
// ============================================================================

/**
 * @brief Axis-aligned bounding box in 3D
 */
struct BoundingBox3D {
    Point3D center;
    Float length{0.0};  ///< Extent along X axis
    Float width{0.0};   ///< Extent along Y axis
    Float height{0.0};  ///< Extent along Z axis
    Float yaw{0.0};     ///< Rotation around Z axis (radians)

    /// Get the 8 corner points of the bounding box
    [[nodiscard]] std::array<Point3D, 8> corners() const noexcept;

    /// Compute volume
    [[nodiscard]] constexpr Float volume() const noexcept { return length * width * height; }

    /// Check if a point is inside the bounding box (ignoring rotation)
    [[nodiscard]] bool contains(const Point3D& point) const noexcept;
};

// ============================================================================
// Sensor Data Types
// ============================================================================

/**
 * @brief LiDAR point with intensity and additional attributes
 */
struct LidarPoint {
    Float x{0.0};
    Float y{0.0};
    Float z{0.0};
    Float intensity{0.0};         ///< Reflectivity [0, 1]
    std::uint16_t ring{0};        ///< Laser ring/channel number
    Float timestamp_offset{0.0};  ///< Time offset from scan start (seconds)

    [[nodiscard]] Point3D toPoint3D() const noexcept { return {x, y, z}; }
};

/**
 * @brief Radar detection point
 */
struct RadarPoint {
    Float range{0.0};            ///< Distance to target (meters)
    Float azimuth{0.0};          ///< Horizontal angle (radians)
    Float elevation{0.0};        ///< Vertical angle (radians)
    Float radial_velocity{0.0};  ///< Velocity towards/away from sensor (m/s)
    Float rcs{0.0};              ///< Radar cross-section (dBsm)
    Float snr{0.0};              ///< Signal-to-noise ratio (dB)

    /// Convert polar coordinates to Cartesian Point3D
    [[nodiscard]] Point3D toPoint3D() const noexcept {
        const Float cos_el = std::cos(elevation);
        return {range * std::cos(azimuth) * cos_el, range * std::sin(azimuth) * cos_el,
                range * std::sin(elevation)};
    }
};

/**
 * @brief GPS/GNSS position with uncertainty
 */
struct GpsPosition {
    Float latitude{0.0};             ///< Degrees
    Float longitude{0.0};            ///< Degrees
    Float altitude{0.0};             ///< Meters above ellipsoid
    Float horizontal_accuracy{0.0};  ///< Meters (1-sigma)
    Float vertical_accuracy{0.0};    ///< Meters (1-sigma)
    std::uint8_t fix_type{0};        ///< 0=none, 1=2D, 2=3D, 3=DGPS, 4=RTK

    /// Check if fix is valid
    [[nodiscard]] constexpr bool isValid() const noexcept { return fix_type >= 2; }
};

/**
 * @brief IMU measurement
 */
struct ImuData {
    Vector3D linear_acceleration;  ///< m/s^2
    Vector3D angular_velocity;     ///< rad/s
    Quaternion orientation;        ///< Optional orientation estimate
    bool orientation_valid{false};
};

// ============================================================================
// Detection and Tracking Types
// ============================================================================

/**
 * @brief Object class enumeration
 */
enum class ObjectClass : std::uint8_t {
    Unknown = 0,
    Car = 1,
    Truck = 2,
    Bus = 3,
    Motorcycle = 4,
    Bicycle = 5,
    Pedestrian = 6,
    Animal = 7,
    TrafficSign = 8,
    TrafficLight = 9,
    Obstacle = 10,
    RoadMarking = 11
};

/**
 * @brief String representation of object class
 */
[[nodiscard]] constexpr const char* toString(ObjectClass obj_class) noexcept {
    switch (obj_class) {
        case ObjectClass::Unknown:
            return "Unknown";
        case ObjectClass::Car:
            return "Car";
        case ObjectClass::Truck:
            return "Truck";
        case ObjectClass::Bus:
            return "Bus";
        case ObjectClass::Motorcycle:
            return "Motorcycle";
        case ObjectClass::Bicycle:
            return "Bicycle";
        case ObjectClass::Pedestrian:
            return "Pedestrian";
        case ObjectClass::Animal:
            return "Animal";
        case ObjectClass::TrafficSign:
            return "TrafficSign";
        case ObjectClass::TrafficLight:
            return "TrafficLight";
        case ObjectClass::Obstacle:
            return "Obstacle";
        case ObjectClass::RoadMarking:
            return "RoadMarking";
    }
    return "Unknown";
}

/**
 * @brief Detected object with tracking information
 */
struct DetectedObject {
    std::uint64_t id{0};  ///< Unique object ID
    ObjectClass object_class{ObjectClass::Unknown};
    Float confidence{0.0};  ///< Detection confidence [0, 1]
    BoundingBox3D bounding_box;
    Vector3D velocity;
    Vector3D acceleration;
    bool velocity_valid{false};
    bool acceleration_valid{false};
};

// ============================================================================
// Eigen Type Aliases (conditional)
// ============================================================================

#ifdef AURA_HAS_EIGEN
/// 3D vector using Eigen
using EigenVector3 = Eigen::Vector3d;

/// 4x4 transformation matrix
using EigenMatrix4 = Eigen::Matrix4d;

/// Dynamic-size matrix
using EigenMatrixX = Eigen::MatrixXd;

/// Dynamic-size vector
using EigenVectorX = Eigen::VectorXd;
#endif

}  // namespace aura
