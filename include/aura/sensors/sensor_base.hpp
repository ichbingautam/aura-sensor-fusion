/**
 * @file sensor_base.hpp
 * @brief Base classes for sensor adapters
 *
 * Provides abstract base classes and interfaces for sensor data handling.
 * All sensor adapters inherit from these classes to provide a unified
 * interface for the fusion pipeline.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/timestamp.hpp>
#include <aura/core/types.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aura {

/**
 * @brief Sensor status enumeration
 */
enum class SensorStatus : std::uint8_t {
    Unknown = 0,
    Initializing = 1,
    Active = 2,
    Degraded = 3,
    Error = 4,
    Disconnected = 5
};

/**
 * @brief Convert sensor status to string
 */
[[nodiscard]] constexpr std::string_view toString(SensorStatus status) noexcept {
    switch (status) {
        case SensorStatus::Unknown:
            return "Unknown";
        case SensorStatus::Initializing:
            return "Initializing";
        case SensorStatus::Active:
            return "Active";
        case SensorStatus::Degraded:
            return "Degraded";
        case SensorStatus::Error:
            return "Error";
        case SensorStatus::Disconnected:
            return "Disconnected";
    }
    return "Unknown";
}

/**
 * @brief Sensor type enumeration
 */
enum class SensorType : std::uint8_t {
    Unknown = 0,
    Lidar = 1,
    Camera = 2,
    Radar = 3,
    Gps = 4,
    Imu = 5,
    Ultrasonic = 6,
    Custom = 255
};

/**
 * @brief Convert sensor type to string
 */
[[nodiscard]] constexpr std::string_view toString(SensorType type) noexcept {
    switch (type) {
        case SensorType::Unknown:
            return "Unknown";
        case SensorType::Lidar:
            return "Lidar";
        case SensorType::Camera:
            return "Camera";
        case SensorType::Radar:
            return "Radar";
        case SensorType::Gps:
            return "Gps";
        case SensorType::Imu:
            return "Imu";
        case SensorType::Ultrasonic:
            return "Ultrasonic";
        case SensorType::Custom:
            return "Custom";
    }
    return "Unknown";
}

/**
 * @brief Sensor configuration parameters
 */
struct SensorConfig {
    std::string name;
    SensorType type{SensorType::Unknown};
    SensorId id{0};
    Float publish_rate_hz{0.0};  ///< Expected data rate in Hz
    Pose3D extrinsics;           ///< Sensor pose relative to vehicle frame

    /// Configuration-specific parameters as key-value pairs
    std::unordered_map<std::string, std::string> parameters;

    /// Get a parameter with default value
    [[nodiscard]] std::string getParam(const std::string& key,
                                       const std::string& default_val = "") const {
        auto it = parameters.find(key);
        return it != parameters.end() ? it->second : default_val;
    }
};

/**
 * @brief Base class for all sensor data
 *
 * All sensor data types inherit from this class to provide common
 * metadata (timestamp, sensor ID, sequence number).
 */
class SensorDataBase {
public:
    virtual ~SensorDataBase() = default;

    /// Get timestamp of data acquisition
    [[nodiscard]] virtual Timestamp timestamp() const noexcept = 0;

    /// Get sensor ID
    [[nodiscard]] virtual SensorId sensorId() const noexcept = 0;

    /// Get sequence number
    [[nodiscard]] virtual SequenceNumber sequenceNumber() const noexcept = 0;

    /// Get frame ID (coordinate frame name)
    [[nodiscard]] virtual std::string_view frameId() const noexcept = 0;

    /// Check if data is valid
    [[nodiscard]] virtual bool isValid() const noexcept = 0;

    /// Clone the data (deep copy)
    [[nodiscard]] virtual std::unique_ptr<SensorDataBase> clone() const = 0;
};

/**
 * @brief Templated sensor data container
 *
 * @tparam T The actual data type (e.g., std::vector<LidarPoint>)
 */
template <typename T>
class SensorData : public SensorDataBase {
public:
    using DataType = T;

    SensorData() = default;

    SensorData(T data, Timestamp ts, SensorId sensor_id, SequenceNumber seq_num,
               std::string frame_id = "sensor")
        : data_(std::move(data)), timestamp_(ts), sensor_id_(sensor_id), sequence_number_(seq_num),
          frame_id_(std::move(frame_id)) {}

    // Move semantics for efficient data transfer
    SensorData(SensorData&&) noexcept = default;
    SensorData& operator=(SensorData&&) noexcept = default;

    // Copy semantics
    SensorData(const SensorData&) = default;
    SensorData& operator=(const SensorData&) = default;

    ~SensorData() override = default;

    /// Access the underlying data
    [[nodiscard]] T& data() noexcept { return data_; }
    [[nodiscard]] const T& data() const noexcept { return data_; }

    /// Move out the data
    [[nodiscard]] T extractData() noexcept { return std::move(data_); }

    // SensorDataBase interface implementation
    [[nodiscard]] Timestamp timestamp() const noexcept override { return timestamp_; }
    [[nodiscard]] SensorId sensorId() const noexcept override { return sensor_id_; }
    [[nodiscard]] SequenceNumber sequenceNumber() const noexcept override {
        return sequence_number_;
    }
    [[nodiscard]] std::string_view frameId() const noexcept override { return frame_id_; }
    [[nodiscard]] bool isValid() const noexcept override { return !timestamp_.isZero(); }

    [[nodiscard]] std::unique_ptr<SensorDataBase> clone() const override {
        return std::make_unique<SensorData<T>>(*this);
    }

    /// Set timestamp
    void setTimestamp(Timestamp ts) noexcept { timestamp_ = ts; }

    /// Set sensor ID
    void setSensorId(SensorId id) noexcept { sensor_id_ = id; }

    /// Set sequence number
    void setSequenceNumber(SequenceNumber seq) noexcept { sequence_number_ = seq; }

    /// Set frame ID
    void setFrameId(std::string frame_id) { frame_id_ = std::move(frame_id); }

private:
    T data_{};
    Timestamp timestamp_;
    SensorId sensor_id_{0};
    SequenceNumber sequence_number_{0};
    std::string frame_id_{"sensor"};
};

// Common sensor data type aliases
using LidarData = SensorData<std::vector<LidarPoint>>;
using RadarData = SensorData<std::vector<RadarPoint>>;
using GpsData = SensorData<GpsPosition>;
using ImuDataPacket = SensorData<ImuData>;

/**
 * @brief Abstract base class for sensor adapters
 *
 * Sensor adapters convert raw sensor data into the normalized format
 * used by the fusion pipeline.
 */
class SensorAdapter {
public:
    using DataCallback = std::function<void(std::unique_ptr<SensorDataBase>)>;

    explicit SensorAdapter(SensorConfig config) : config_(std::move(config)) {}

    virtual ~SensorAdapter() = default;

    // Non-copyable, movable
    SensorAdapter(const SensorAdapter&) = delete;
    SensorAdapter& operator=(const SensorAdapter&) = delete;
    SensorAdapter(SensorAdapter&&) noexcept = default;
    SensorAdapter& operator=(SensorAdapter&&) noexcept = default;

    /// Initialize the sensor
    virtual bool initialize() = 0;

    /// Start data acquisition
    virtual bool start() = 0;

    /// Stop data acquisition
    virtual void stop() = 0;

    /// Check if sensor is running
    [[nodiscard]] virtual bool isRunning() const noexcept = 0;

    /// Get current sensor status
    [[nodiscard]] virtual SensorStatus status() const noexcept = 0;

    /// Set the data callback
    void setCallback(DataCallback callback) { callback_ = std::move(callback); }

    /// Get sensor configuration
    [[nodiscard]] const SensorConfig& config() const noexcept { return config_; }

    /// Get sensor name
    [[nodiscard]] std::string_view name() const noexcept { return config_.name; }

    /// Get sensor type
    [[nodiscard]] SensorType type() const noexcept { return config_.type; }

    /// Get sensor ID
    [[nodiscard]] SensorId id() const noexcept { return config_.id; }

    /// Get sensor extrinsics
    [[nodiscard]] const Pose3D& extrinsics() const noexcept { return config_.extrinsics; }

protected:
    /// Called by implementations to deliver data
    void deliverData(std::unique_ptr<SensorDataBase> data) {
        if (callback_) {
            callback_(std::move(data));
        }
    }

    SensorConfig config_;
    DataCallback callback_;
};

/**
 * @brief Factory for creating sensor adapters
 */
class SensorFactory {
public:
    using CreatorFunc = std::function<std::unique_ptr<SensorAdapter>(const SensorConfig&)>;

    /// Register a sensor adapter creator
    static void registerAdapter(SensorType type, CreatorFunc creator) {
        creators()[type] = std::move(creator);
    }

    /// Create a sensor adapter
    [[nodiscard]] static std::unique_ptr<SensorAdapter> create(const SensorConfig& config) {
        auto it = creators().find(config.type);
        if (it != creators().end()) {
            return it->second(config);
        }
        return nullptr;
    }

private:
    static std::unordered_map<SensorType, CreatorFunc>& creators() {
        static std::unordered_map<SensorType, CreatorFunc> instance;
        return instance;
    }
};

}  // namespace aura
