/**
 * @file extended_kalman_filter.hpp
 * @brief Extended Kalman Filter implementation for non-linear systems
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/math/matrix.hpp>

#include <functional>

namespace aura {
namespace filters {

/**
 * @brief Extended Kalman Filter implementation
 *
 * @tparam StateDim Dimension of the state vector
 * @tparam MeasureDim Dimension of the measurement vector
 * @tparam T Numeric type (default double)
 */
template <size_t StateDim, size_t MeasureDim, typename T = double>
class ExtendedKalmanFilter {
public:
    using StateVector = math::Vector<T, StateDim>;
    using StateMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureVector = math::Vector<T, MeasureDim>;
    using MeasureMatrix = math::Matrix<T, MeasureDim, StateDim>;
    using ProcessNoiseMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureNoiseMatrix = math::Matrix<T, MeasureDim, MeasureDim>;

    ExtendedKalmanFilter()
        : x_(StateVector::Zero()), P_(StateMatrix::Identity()), Q_(ProcessNoiseMatrix::Identity()),
          R_(MeasureNoiseMatrix::Identity()) {}

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
     * @brief Set the Process Noise Covariance
     * @param Q Process noise covariance matrix
     */
    void setProcessNoise(const ProcessNoiseMatrix& Q) { Q_ = Q; }

    /**
     * @brief Set the Measurement Noise Covariance
     * @param R Measurement noise covariance matrix
     */
    void setMeasurementNoise(const MeasureNoiseMatrix& R) { R_ = R; }

    /**
     * @brief Prediction step
     * Projects the state and error covariance ahead using non-linear transition.
     * @param f State transition function: x_k = f(x_{k-1})
     * @param F Jacobian of f evaluated at current state x_{k-1}
     */
    void predict(const std::function<StateVector(const StateVector&)>& f, const StateMatrix& F) {
        // x = f(x)
        x_ = f(x_);
        // P = F * P * F^T + Q
        P_ = F * P_ * F.transpose() + Q_;
    }

    /**
     * @brief Measurement Update step
     * Adjusts the projected state based on the measurement using non-linear function.
     * @param z Measurement vector
     * @param h Measurement function: z_k = h(x_k)
     * @param H Jacobian of h evaluated at current predicted state x_k
     */
    void update(const MeasureVector& z, const std::function<MeasureVector(const StateVector&)>& h,
                const MeasureMatrix& H) {
        // z_pred = h(x)
        MeasureVector z_pred = h(x_);
        // y = z - z_pred
        MeasureVector y = z - z_pred;

        // S = H * P * H^T + R
        MeasureNoiseMatrix S = H * P_ * H.transpose() + R_;

        // K = P * H^T * S^-1
        math::Matrix<T, StateDim, MeasureDim> K = P_ * H.transpose() * S.inverse();

        // x = x + K * y
        x_ = x_ + (K * y);

        // P = (I - K * H) * P
        StateMatrix I = StateMatrix::Identity();
        P_ = (I - (K * H)) * P_;
    }

    // Accessors
    [[nodiscard]] const StateVector& getState() const { return x_; }
    [[nodiscard]] const StateMatrix& getCovariance() const { return P_; }

private:
    StateVector x_;
    StateMatrix P_;
    ProcessNoiseMatrix Q_;
    MeasureNoiseMatrix R_;
};

}  // namespace filters
}  // namespace aura
