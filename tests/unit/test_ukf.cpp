/**
 * @file test_ukf.cpp
 * @brief Unit tests for UnscentedKalmanFilter
 */

#include <aura/filters/unscented_kalman_filter.hpp>
#include <aura/math/matrix.hpp>

#include <cmath>

#include <gtest/gtest.h>

using namespace aura::math;
using namespace aura::filters;

class TestUnscentedKalmanFilter : public ::testing::Test {
protected:
    UnscentedKalmanFilter<2, 2> filter_;

    void SetUp() override {
        Matrix<double, 2, 2> q_mat = {0.1, 0.0, 0.0, 0.1};
        filter_.setProcessNoise(q_mat);

        Matrix<double, 2, 2> r_mat = {0.05, 0.0, 0.0, 0.05};
        filter_.setMeasurementNoise(r_mat);

        Vector<double, 2> x0 = {1.0, 0.0};
        Matrix<double, 2, 2> p0 = {1.0, 0.0, 0.0, 1.0};
        filter_.init(x0, p0);
    }
};

TEST_F(TestUnscentedKalmanFilter, PredictAndUpdate) {
    auto f = [](const Vector<double, 2>& x) -> Vector<double, 2> {
        return {x(0, 0), x(1, 0) + x(0, 0) * x(0, 0)};
    };

    filter_.predict(f);

    auto pred_state = filter_.getState();
    EXPECT_NEAR(pred_state(0, 0), 1.0, 1e-3);
    EXPECT_NEAR(pred_state(1, 0), 2.0, 1e-3);

    auto h = [](const Vector<double, 2>& x) -> Vector<double, 2> {
        double r = std::sqrt(x(0, 0) * x(0, 0) + x(1, 0) * x(1, 0));
        double theta = std::atan2(x(1, 0), x(0, 0));
        return {r, theta};
    };

    Vector<double, 2> z = {1.5, 0.8};

    filter_.update(z, h);

    auto updated_state = filter_.getState();
    EXPECT_NE(updated_state(0, 0), 1.0);
    EXPECT_NE(updated_state(1, 0), 1.0);
}
