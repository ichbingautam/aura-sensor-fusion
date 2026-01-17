/**
 * @file fusion_concepts.hpp
 * @brief C++20 concepts for fusion algorithms
 *
 * Defines concept-based constraints for fusion algorithms,
 * ensuring type safety at compile time.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/types.hpp>
#include <aura/sensors/sensor_base.hpp>

#include <concepts>
#include <type_traits>

namespace aura {

// Forward declarations
class FusedData;

/**
 * @brief Concept for types that can be fused
 *
 * A type is fuseable if it provides timestamp and validity information.
 */
template <typename T>
concept Fuseable = requires(T data) {
    { data.timestamp() } -> std::convertible_to<Timestamp>;
    { data.isValid() } -> std::convertible_to<bool>;
};

/**
 * @brief Concept for fusion algorithms
 *
 * A fusion algorithm must be able to fuse two data inputs and
 * provide a confidence measure.
 */
template <typename Algorithm>
concept FusionAlgorithm = requires(Algorithm algo, std::unique_ptr<SensorDataBase> d1,
                                   std::unique_ptr<SensorDataBase> d2) {
    // Must have a fuse method
    { algo.fuse(d1, d2) } -> std::convertible_to<std::unique_ptr<SensorDataBase>>;

    // Must provide confidence measurement
    { algo.confidence() } -> std::floating_point;
};

/**
 * @brief Concept for prediction algorithms
 *
 * Prediction algorithms extrapolate state forward in time.
 */
template <typename Algorithm>
concept PredictionAlgorithm =
    requires(Algorithm algo, const SensorDataBase& data, Timestamp target) {
        { algo.predict(data, target) } -> std::convertible_to<std::unique_ptr<SensorDataBase>>;
    };

/**
 * @brief Concept for update algorithms
 *
 * Update algorithms incorporate new measurements into state.
 */
template <typename Algorithm>
concept UpdateAlgorithm =
    requires(Algorithm algo, SensorDataBase& state, const SensorDataBase& measurement) {
        { algo.update(state, measurement) } -> std::convertible_to<bool>;
    };

/**
 * @brief Concept for complete filter algorithms (predict + update)
 */
template <typename Algorithm>
concept FilterAlgorithm = PredictionAlgorithm<Algorithm> && UpdateAlgorithm<Algorithm>;

/**
 * @brief Concept for data association algorithms
 *
 * Associates measurements with existing tracks.
 */
template <typename Algorithm, typename Track, typename Measurement>
concept AssociationAlgorithm = requires(Algorithm algo, const std::vector<Track>& tracks,
                                        const std::vector<Measurement>& measurements) {
    // Returns association matrix or pairs
    { algo.associate(tracks, measurements) };
};

/**
 * @brief Concept for track management
 */
template <typename T>
concept Trackable = requires(T track) {
    { track.id() } -> std::convertible_to<std::uint64_t>;
    { track.state() } -> Fuseable;
    { track.covariance() };
    { track.age() } -> std::convertible_to<int>;
};

/**
 * @brief Concept for processing nodes in the pipeline
 */
template <typename Node>
concept ProcessingNode = requires(Node node, std::unique_ptr<SensorDataBase> input) {
    { node.process(std::move(input)) } -> std::convertible_to<std::unique_ptr<SensorDataBase>>;
    { node.name() } -> std::convertible_to<std::string_view>;
    { node.initialize() } -> std::convertible_to<bool>;
};

/**
 * @brief Concept for configurable components
 */
template <typename T, typename Config>
concept Configurable = requires(T obj, const Config& config) {
    { T(config) };
    { obj.configure(config) } -> std::convertible_to<bool>;
};

/**
 * @brief Concept for serializable data
 */
template <typename T>
concept Serializable = requires(T obj, std::vector<std::byte>& buffer) {
    { obj.serialize(buffer) } -> std::convertible_to<bool>;
    { T::deserialize(buffer) } -> std::same_as<std::optional<T>>;
};

/**
 * @brief Constraint for matrix-like types
 */
template <typename T>
concept MatrixLike = requires(T m, std::size_t i, std::size_t j) {
    { m(i, j) } -> std::convertible_to<double>;
    { m.rows() } -> std::convertible_to<std::size_t>;
    { m.cols() } -> std::convertible_to<std::size_t>;
};

/**
 * @brief Constraint for vector-like types
 */
template <typename T>
concept VectorLike = requires(T v, std::size_t i) {
    { v(i) } -> std::convertible_to<double>;
    { v.size() } -> std::convertible_to<std::size_t>;
};

}  // namespace aura
