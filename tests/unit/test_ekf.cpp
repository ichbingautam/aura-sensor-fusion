/**
 * @file test_ekf.cpp
 * @brief Unit tests for ExtendedKalmanFilter
 */

#include <aura/filters/extended_kalman_filter.hpp>
#include <aura/math/matrix.hpp>

#include <cmath>

#include <gtest/gtest.h>

using namespace aura::math;
using namespace aura::filters;

class TestExtendedKalmanFilter : public ::testing::Test {
protected:
    // 2D State (x, y), 2D Measurement (r, theta)
    ExtendedKalmanFilter<2, 2> ekf;

    void SetUp() override {
        Matrix<double, 2, 2> Q = {0.1, 0.0, 0.0, 0.1};
        ekf.setProcessNoise(Q);

        Matrix<double, 2, 2> R = {0.05, 0.0, 0.0, 0.05};
        ekf.setMeasurementNoise(R);

        Vector<double, 2> x0 = {1.0, 0.0};
        Matrix<double, 2, 2> P0 = {1.0, 0.0, 0.0, 1.0};
        ekf.init(x0, P0);
    }
};

TEST_F(TestExtendedKalmanFilter, PredictAndUpdate) {
    // Non-linear state transition f(x) = [x_0, x_1 + x_0^2]
    auto f = [](const Vector<double, 2>& x) -> Vector<double, 2> {
        return {x(0, 0), x(1, 0) + x(0, 0) * x(0, 0)};
    };

    // Jacobian of f: F = [1, 0; 2*x_0, 1]
    auto state = ekf.getState();
    Matrix<double, 2, 2> F = {1.0, 0.0, 2.0 * state(0, 0), 1.0};

    ekf.predict(f, F);

    auto pred_state = ekf.getState();
    EXPECT_DOUBLE_EQ(pred_state(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(pred_state(1, 0), 1.0);

    // Non-linear measurement h(x) = [sqrt(x_0^2 + x_1^2), atan2(x_1, x_0)]
    auto h = [](const Vector<double, 2>& x) -> Vector<double, 2> {
        double r = std::sqrt(x(0, 0) * x(0, 0) + x(1, 0) * x(1, 0));
        double theta = std::atan2(x(1, 0), x(0, 0));
        return {r, theta};
    };

    // Jacobian of h
    double r_sq = pred_state(0, 0) * pred_state(0, 0) + pred_state(1, 0) * pred_state(1, 0);
    double r = std::sqrt(r_sq);
    Matrix<double, 2, 2> H = {pred_state(0, 0) / r, pred_state(1, 0) / r, -pred_state(1, 0) / r_sq,
                              pred_state(0, 0) / r_sq};

    // Simulated measurement
    Vector<double, 2> z = {1.5, 0.8};  // r = 1.5, theta = 0.8 rad

    ekf.update(z, h, H);

    auto updated_state = ekf.getState();
    // After update, state should move somewhat towards the measurement mapping
    EXPECT_NE(updated_state(0, 0), 1.0);
    EXPECT_NE(updated_state(1, 0), 1.0);
}
