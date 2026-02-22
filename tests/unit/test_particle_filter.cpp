/**
 * @file test_particle_filter.cpp
 * @brief Unit tests for ParticleFilter
 */

#include <aura/filters/particle_filter.hpp>
#include <aura/math/matrix.hpp>

#include <cmath>

#include <gtest/gtest.h>

using namespace aura::math;
using namespace aura::filters;

class TestParticleFilter : public ::testing::Test {
protected:
    ParticleFilter<2, 2, 1000> filter_;

    void SetUp() override {
        Vector<double, 2> x0 = {1.0, 0.0};
        Matrix<double, 2, 2> p0 = {0.1, 0.0, 0.0, 0.1};
        filter_.init(x0, p0);
    }
};

TEST_F(TestParticleFilter, PredictUpdateResample) {
    // Prediction with noise
    auto f = [](const Vector<double, 2>& x, std::mt19937& gen) -> Vector<double, 2> {
        std::normal_distribution<double> dist(0.0, 0.1);
        Vector<double, 2> next_x;
        next_x(0, 0) = x(0, 0) + dist(gen);
        next_x(1, 0) = x(1, 0) + x(0, 0) * x(0, 0) + dist(gen);
        return next_x;
    };

    filter_.predict(f);

    auto pred_state = filter_.getState();
    EXPECT_NEAR(pred_state(0, 0), 1.0, 0.1);
    EXPECT_NEAR(pred_state(1, 0), 1.0, 0.2);

    // Update with weight
    auto h_weight = [](const Vector<double, 2>& z, const Vector<double, 2>& x) -> double {
        double r = std::sqrt(x(0, 0) * x(0, 0) + x(1, 0) * x(1, 0));
        double theta = std::atan2(x(1, 0), x(0, 0));

        // Compute pdf assuming Gaussian measurement noise R
        double diff_r = z(0, 0) - r;
        double diff_t = z(1, 0) - theta;
        double var_r = 0.05;
        double var_t = 0.05;

        return std::exp(-0.5 * (diff_r * diff_r / var_r + diff_t * diff_t / var_t));
    };

    Vector<double, 2> z = {1.5, 0.8};
    filter_.update(z, h_weight);

    auto updated_state = filter_.getState();
    EXPECT_NE(updated_state(0, 0), pred_state(0, 0));

    // Resample
    filter_.resample();

    // Check state after resample (should be very similar to before resampling)
    auto resampled_state = filter_.getState();
    EXPECT_NEAR(resampled_state(0, 0), updated_state(0, 0), 0.05);
    EXPECT_NEAR(resampled_state(1, 0), updated_state(1, 0), 0.05);
}
