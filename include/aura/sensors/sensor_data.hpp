/**
 * @file sensor_data.hpp
 * @brief Type-safe sensor data adapter with transformations
 *
 * Provides a template-based adapter for transforming sensor data
 * using the sensor traits system.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/timestamp.hpp>
#include <aura/core/types.hpp>
#include <aura/sensors/sensor_base.hpp>
#include <aura/sensors/sensor_traits.hpp>

#include <functional>
#include <memory>
#include <utility>

namespace aura {

/**
 * @brief Type-safe sensor data adapter with transformation support
 *
 * This adapter wraps raw sensor data and provides methods for
 * transforming it to normalized representations.
 *
 * @tparam SensorT The sensor type (must satisfy ValidSensor concept)
 * @tparam DataT The actual data type (defaults to raw_type from traits)
 */
template <ValidSensor SensorT, typename DataT = RawDataType<SensorT>>
class SensorDataAdapter {
public:
    using SensorType = SensorT;
    using DataType = DataT;
    using NormalizedType = NormalizedDataType<SensorT>;

    SensorDataAdapter() = default;

    /**
     * @brief Construct adapter with data
     *
     * @param data The sensor data
     * @param timestamp Data timestamp
     * @param sensor_id Sensor identifier
     */
    SensorDataAdapter(DataT data, Timestamp timestamp, SensorId sensor_id)
        : data_(std::move(data)), timestamp_(timestamp), sensor_id_(sensor_id) {}

    // Move semantics
    SensorDataAdapter(SensorDataAdapter&&) noexcept = default;
    SensorDataAdapter& operator=(SensorDataAdapter&&) noexcept = default;

    // Copy semantics
    SensorDataAdapter(const SensorDataAdapter&) = default;
    SensorDataAdapter& operator=(const SensorDataAdapter&) = default;

    ~SensorDataAdapter() = default;

    /**
     * @brief Transform the data using a callback
     *
     * @tparam Callback Callable type
     * @param fn Function to apply to the data
     * @return Result of applying fn to the data
     */
    template <typename Callback>
    [[nodiscard]] auto transform(Callback&& fn) -> decltype(auto) {
        return std::forward<Callback>(fn)(data_);
    }

    /**
     * @brief Transform the data (const version)
     */
    template <typename Callback>
    [[nodiscard]] auto transform(Callback&& fn) const -> decltype(auto) {
        return std::forward<Callback>(fn)(data_);
    }

    /**
     * @brief Map the data to a new type
     *
     * @tparam Callback Callable type
     * @param fn Function to apply to the data
     * @return New adapter with transformed data
     */
    template <typename Callback>
    [[nodiscard]] auto map(Callback&& fn) const {
        using ResultType = std::invoke_result_t<Callback, const DataT&>;
        return SensorDataAdapter<SensorT, ResultType>(std::forward<Callback>(fn)(data_), timestamp_,
                                                      sensor_id_);
    }

    /**
     * @brief Filter data elements (for container data types)
     *
     * @tparam Predicate Callable type returning bool
     * @param pred Predicate function
     * @return New adapter with filtered data
     */
    template <typename Predicate>
        requires requires(DataT d) {
            d.begin();
            d.end();
            d.push_back(std::declval<typename DataT::value_type>());
        }
    [[nodiscard]] auto filter(Predicate&& pred) const {
        DataT filtered;
        for (const auto& elem : data_) {
            if (std::forward<Predicate>(pred)(elem)) {
                filtered.push_back(elem);
            }
        }
        return SensorDataAdapter<SensorT, DataT>(std::move(filtered), timestamp_, sensor_id_);
    }

    /// Access the underlying data
    [[nodiscard]] DataT& data() noexcept { return data_; }
    [[nodiscard]] const DataT& data() const noexcept { return data_; }

    /// Get timestamp
    [[nodiscard]] Timestamp timestamp() const noexcept { return timestamp_; }

    /// Get sensor ID
    [[nodiscard]] SensorId sensorId() const noexcept { return sensor_id_; }

    /// Check if data is valid
    [[nodiscard]] bool isValid() const noexcept { return !timestamp_.isZero(); }

private:
    DataT data_{};
    Timestamp timestamp_;
    SensorId sensor_id_{0};
};

/**
 * @brief Create a sensor data adapter
 *
 * @tparam SensorT Sensor type
 * @tparam DataT Data type
 * @param data The sensor data
 * @param timestamp Data timestamp
 * @param sensor_id Sensor identifier
 * @return SensorDataAdapter instance
 */
template <ValidSensor SensorT, typename DataT>
[[nodiscard]] auto makeSensorData(DataT&& data, Timestamp timestamp, SensorId sensor_id) {
    return SensorDataAdapter<SensorT, std::decay_t<DataT>>(std::forward<DataT>(data), timestamp,
                                                           sensor_id);
}

/**
 * @brief Combine multiple sensor data adapters
 *
 * Useful for multi-sensor fusion operations.
 *
 * @tparam Adapters Adapter types
 */
template <typename... Adapters>
class SensorDataPack {
public:
    SensorDataPack(Adapters&&... adapters) : adapters_(std::forward<Adapters>(adapters)...) {}

    /// Get the number of adapters
    [[nodiscard]] static constexpr std::size_t size() noexcept { return sizeof...(Adapters); }

    /// Get adapter at index
    template <std::size_t I>
    [[nodiscard]] auto& get() noexcept {
        return std::get<I>(adapters_);
    }

    template <std::size_t I>
    [[nodiscard]] const auto& get() const noexcept {
        return std::get<I>(adapters_);
    }

    /// Get the tuple of adapters
    [[nodiscard]] auto& data() noexcept { return adapters_; }
    [[nodiscard]] const auto& data() const noexcept { return adapters_; }

    /// Apply a function to all adapters
    template <typename Func>
    void forEach(Func&& func) {
        std::apply([&](auto&... args) { (func(args), ...); }, adapters_);
    }

    /// Get the latest timestamp among all adapters
    [[nodiscard]] Timestamp latestTimestamp() const {
        Timestamp latest;
        std::apply(
            [&](const auto&... args) {
                ((args.timestamp() > latest ? latest = args.timestamp() : latest), ...);
            },
            adapters_);
        return latest;
    }

    /// Get the earliest timestamp among all adapters
    [[nodiscard]] Timestamp earliestTimestamp() const {
        Timestamp earliest = Timestamp::max();
        std::apply(
            [&](const auto&... args) {
                ((args.timestamp() < earliest ? earliest = args.timestamp() : earliest), ...);
            },
            adapters_);
        return earliest;
    }

private:
    std::tuple<Adapters...> adapters_;
};

/**
 * @brief Create a sensor data pack
 */
template <typename... Adapters>
[[nodiscard]] auto makeSensorPack(Adapters&&... adapters) {
    return SensorDataPack<std::decay_t<Adapters>...>(std::forward<Adapters>(adapters)...);
}

}  // namespace aura
