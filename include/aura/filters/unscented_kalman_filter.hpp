/**
 * @file unscented_kalman_filter.hpp
 * @brief Unscented Kalman Filter implementation
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/math/matrix.hpp>

#include <functional>

namespace aura {
namespace filters {

/**
 * @brief Unscented Kalman Filter implementation
 *
 * @tparam StateDim Dimension of the state vector
 * @tparam MeasureDim Dimension of the measurement vector
 * @tparam T Numeric type (default double)
 */
template <size_t StateDim, size_t MeasureDim, typename T = double>
class UnscentedKalmanFilter {
public:
    using StateVector = math::Vector<T, StateDim>;
    using StateMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureVector = math::Vector<T, MeasureDim>;
    using MeasureMatrix = math::Matrix<T, MeasureDim, MeasureDim>;
    using ProcessNoiseMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureNoiseMatrix = math::Matrix<T, MeasureDim, MeasureDim>;
    using SigmaPoints = math::Matrix<T, StateDim, 2 * StateDim + 1>;
    using MeasureSigmaPoints = math::Matrix<T, MeasureDim, 2 * StateDim + 1>;
    using WeightsVector = math::Vector<T, 2 * StateDim + 1>;

    UnscentedKalmanFilter(T alpha = 1e-3, T beta = 2.0, T kappa = 0.0)
        : x_(StateVector::Zero()), P_(StateMatrix::Identity()), Q_(ProcessNoiseMatrix::Identity()),
          R_(MeasureNoiseMatrix::Identity()), alpha_(alpha), beta_(beta), kappa_(kappa) {
        lambda_ = alpha_ * alpha_ * (StateDim + kappa_) - StateDim;

        // Initialize weights
        weights_mean_(0, 0) = lambda_ / (StateDim + lambda_);
        weights_cov_(0, 0) = (lambda_ / (StateDim + lambda_)) + (1.0 - alpha_ * alpha_ + beta_);

        for (size_t i = 1; i < 2 * StateDim + 1; ++i) {
            weights_mean_(i, 0) = 1.0 / (2.0 * (StateDim + lambda_));
            weights_cov_(i, 0) = weights_mean_(i, 0);
        }
    }

    void init(const StateVector& initial_state, const StateMatrix& initial_covariance) {
        x_ = initial_state;
        P_ = initial_covariance;
    }

    void setProcessNoise(const ProcessNoiseMatrix& q) { Q_ = q; }
    void setMeasurementNoise(const MeasureNoiseMatrix& r) { R_ = r; }

    void predict(const std::function<StateVector(const StateVector&)>& f) {
        // 1. Generate sigma points
        SigmaPoints x_sigmas = generateSigmaPoints(x_, P_);

        // 2. Predict sigma points
        for (size_t i = 0; i < 2 * StateDim + 1; ++i) {
            StateVector v;
            for (size_t j = 0; j < StateDim; ++j) {
                v(j, 0) = x_sigmas(j, i);
            }
            StateVector v_pred = f(v);
            for (size_t j = 0; j < StateDim; ++j) {
                x_sigmas_pred_(j, i) = v_pred(j, 0);
            }
        }

        // 3. Calculate predicted state mean
        x_ = StateVector::Zero();
        for (size_t i = 0; i < 2 * StateDim + 1; ++i) {
            for (size_t j = 0; j < StateDim; ++j) {
                x_(j, 0) += weights_mean_(i, 0) * x_sigmas_pred_(j, i);
            }
        }

        // 4. Calculate predicted state covariance
        P_ = Q_;
        for (size_t i = 0; i < 2 * StateDim + 1; ++i) {
            StateVector diff;
            for (size_t j = 0; j < StateDim; ++j) {
                diff(j, 0) = x_sigmas_pred_(j, i) - x_(j, 0);
            }
            P_ = P_ + (diff * diff.transpose()) * weights_cov_(i, 0);
        }
    }

    void update(const MeasureVector& z, const std::function<MeasureVector(const StateVector&)>& h) {
        // 1. Predict measurement sigma points
        MeasureSigmaPoints z_sigmas;
        for (size_t i = 0; i < 2 * StateDim + 1; ++i) {
            StateVector v;
            for (size_t j = 0; j < StateDim; ++j) {
                v(j, 0) = x_sigmas_pred_(j, i);
            }
            MeasureVector z_pred = h(v);
            for (size_t j = 0; j < MeasureDim; ++j) {
                z_sigmas(j, i) = z_pred(j, 0);
            }
        }

        // 2. Calculate predicted measurement mean
        MeasureVector z_mean = MeasureVector::Zero();
        for (size_t i = 0; i < 2 * StateDim + 1; ++i) {
            for (size_t j = 0; j < MeasureDim; ++j) {
                z_mean(j, 0) += weights_mean_(i, 0) * z_sigmas(j, i);
            }
        }

        // 3. Calculate innovation covariance (S) and cross covariance (Tc)
        MeasureNoiseMatrix s_mat = R_;
        math::Matrix<T, StateDim, MeasureDim> t_c = math::Matrix<T, StateDim, MeasureDim>::Zero();

        for (size_t i = 0; i < 2 * StateDim + 1; ++i) {
            MeasureVector z_diff;
            for (size_t j = 0; j < MeasureDim; ++j) {
                z_diff(j, 0) = z_sigmas(j, i) - z_mean(j, 0);
            }

            StateVector x_diff;
            for (size_t j = 0; j < StateDim; ++j) {
                x_diff(j, 0) = x_sigmas_pred_(j, i) - x_(j, 0);
            }

            s_mat = s_mat + (z_diff * z_diff.transpose()) * weights_cov_(i, 0);
            t_c = t_c + (x_diff * z_diff.transpose()) * weights_cov_(i, 0);
        }

        // 4. Calculate Kalman gain k_mat
        math::Matrix<T, StateDim, MeasureDim> k_mat = t_c * s_mat.inverse();

        // 5. Update state mean and covariance
        MeasureVector y = z - z_mean;
        x_ = x_ + (k_mat * y);
        P_ = P_ - (k_mat * s_mat * k_mat.transpose());
    }

    [[nodiscard]] const StateVector& getState() const { return x_; }
    [[nodiscard]] const StateMatrix& getCovariance() const { return P_; }

private:
    SigmaPoints generateSigmaPoints(const StateVector& x, const StateMatrix& p_mat) const {
        SigmaPoints sigmas;

        // P_scaled = (n + lambda) * P
        StateMatrix p_scaled = p_mat * (StateDim + lambda_);

        // L = cholesky(P_scaled)
        StateMatrix l_mat = p_scaled.cholesky();

        // set first column to mean
        for (size_t i = 0; i < StateDim; ++i) {
            sigmas(i, 0) = x(i, 0);
        }

        for (size_t i = 0; i < StateDim; ++i) {
            for (size_t j = 0; j < StateDim; ++j) {
                sigmas(j, i + 1) = x(j, 0) + l_mat(j, i);
                sigmas(j, i + 1 + StateDim) = x(j, 0) - l_mat(j, i);
            }
        }

        return sigmas;
    }

    StateVector x_;
    StateMatrix P_;
    ProcessNoiseMatrix Q_;
    MeasureNoiseMatrix R_;

    SigmaPoints x_sigmas_pred_;
    WeightsVector weights_mean_;
    WeightsVector weights_cov_;

    T alpha_;
    T beta_;
    T kappa_;
    T lambda_;
};

}  // namespace filters
}  // namespace aura
