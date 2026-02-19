/**
 * @file kalman_filter.hpp
 * @brief Generic Linear Kalman Filter implementation
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/math/matrix.hpp>

namespace aura {
namespace filters {

/**
 * @brief Standard Linear Kalman Filter implementation
 *
 * @tparam StateDim Dimension of the state vector
 * @tparam MeasureDim Dimension of the measurement vector
 * @tparam T Numeric type (default double)
 */
template <size_t StateDim, size_t MeasureDim, typename T = double>
class KalmanFilter {
public:
    using StateVector = math::Vector<T, StateDim>;
    using StateMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureVector = math::Vector<T, MeasureDim>;
    using MeasureMatrix = math::Matrix<T, MeasureDim, StateDim>;
    using ProcessNoiseMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureNoiseMatrix = math::Matrix<T, MeasureDim, MeasureDim>;

    KalmanFilter()
        : x_(StateVector::Zero()), P_(StateMatrix::Identity()), F_(StateMatrix::Identity()),
          Q_(ProcessNoiseMatrix::Identity()) {}

    /**
     * @brief Initialize the filter state
     * @param initial_state Initial state vector
     * @param initial_covariance Initial error covariance matrix
     */
    void init(const StateVector& initial_state, const StateMatrix& initial_covariance) {
        x_ = initial_state;
        P_ = initial_covariance;
    }

    /**
     * @brief Set the State Transition model
     * @param F State transition matrix
     */
    void setTransitionMatrix(const StateMatrix& F) { F_ = F; }

    /**
     * @brief Set the Process Noise Covariance
     * @param Q Process noise covariance matrix
     */
    void setProcessNoise(const ProcessNoiseMatrix& Q) { Q_ = Q; }

    /**
     * @brief Set the Measurement Model
     * @param H Measurement matrix
     */
    void setMeasurementMatrix(const MeasureMatrix& H) { H_ = H; }

    /**
     * @brief Set the Measurement Noise Covariance
     * @param R Measurement noise covariance matrix
     */
    void setMeasurementNoise(const MeasureNoiseMatrix& R) { R_ = R; }

    /**
     * @brief Prediction step
     * Projects the state and error covariance ahead.
     */
    void predict() {
        // x = F * x
        x_ = F_ * x_;
        // P = F * P * F^T + Q
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    /**
     * @brief Measurement Update step
     * Adjusts the projected state based on the measurement.
     * @param z Measurement vector
     */
    void update(const MeasureVector& z) {
        // y = z - H * x
        MeasureVector y = z - (H_ * x_);

        // S = H * P * H^T + R
        MeasureNoiseMatrix S = H_ * P_ * H_.transpose() + R_;

        // K = P * H^T * S^-1
        math::Matrix<T, StateDim, MeasureDim> K = P_ * H_.transpose() * S.inverse();

        // x = x + K * y
        x_ = x_ + (K * y);

        // P = (I - K * H) * P
        StateMatrix I = StateMatrix::Identity();
        P_ = (I - (K * H_)) * P_;
    }

    // Accessors
    [[nodiscard]] const StateVector& getState() const { return x_; }
    [[nodiscard]] const StateMatrix& getCovariance() const { return P_; }

private:
    // State
    StateVector x_;
    StateMatrix P_;

    // Model
    StateMatrix F_;
    ProcessNoiseMatrix Q_;
    MeasureMatrix H_;
    MeasureNoiseMatrix R_;
};

}  // namespace filters
}  // namespace aura
