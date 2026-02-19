/**
 * @file test_kalman_filter.cpp
 * @brief Unit tests for Matrix and KalmanFilter
 */

#include <aura/filters/kalman_filter.hpp>
#include <aura/math/matrix.hpp>

#include <gtest/gtest.h>

using namespace aura::math;
using namespace aura::filters;

// --- Matrix Tests ---

TEST(MatrixTest, Initialization) {
    Matrix<double, 2, 2> m;
    EXPECT_DOUBLE_EQ(m(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(m(1, 1), 0.0);

    Matrix<double, 2, 2> m2 = {1.0, 2.0, 3.0, 4.0};
    EXPECT_DOUBLE_EQ(m2(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(m2(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(m2(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(m2(1, 1), 4.0);
}

TEST(MatrixTest, Identity) {
    auto I = Matrix<double, 3, 3>::Identity();
    EXPECT_DOUBLE_EQ(I(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(I(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(I(2, 2), 1.0);
    EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
}

TEST(MatrixTest, Transpose) {
    Matrix<double, 2, 3> m = {1, 2, 3, 4, 5, 6};
    auto mt = m.transpose();
    EXPECT_EQ(mt.RowCount, 3);
    EXPECT_EQ(mt.ColCount, 2);
    EXPECT_DOUBLE_EQ(mt(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(mt(0, 1), 4.0);
    EXPECT_DOUBLE_EQ(mt(1, 0), 2.0);
    EXPECT_DOUBLE_EQ(mt(2, 1), 6.0);
}

TEST(MatrixTest, AdditionSubtraction) {
    Matrix<double, 2, 2> A = {1, 2, 3, 4};
    Matrix<double, 2, 2> B = {5, 6, 7, 8};
    auto C = A + B;
    EXPECT_DOUBLE_EQ(C(0, 0), 6.0);
    EXPECT_DOUBLE_EQ(C(1, 1), 12.0);

    auto D = B - A;
    EXPECT_DOUBLE_EQ(D(0, 0), 4.0);
    EXPECT_DOUBLE_EQ(D(1, 1), 4.0);
}

TEST(MatrixTest, Multiplication) {
    Matrix<double, 2, 3> A = {1, 2, 3, 4, 5, 6};
    Matrix<double, 3, 2> B = {7, 8, 9, 1, 2, 3};

    // [1 2 3]   [7 8]   [1*7+2*9+3*2  1*8+2*1+3*3]   [7+18+6   8+2+9 ]   [31 19]
    // [4 5 6] * [9 1] = [4*7+5*9+6*2  4*8+5*1+6*3] = [28+45+12 32+5+18]   [85 55]
    //           [2 3]

    auto C = A * B;
    EXPECT_EQ(C.RowCount, 2);
    EXPECT_EQ(C.ColCount, 2);
    EXPECT_DOUBLE_EQ(C(0, 0), 31.0);
    EXPECT_DOUBLE_EQ(C(0, 1), 19.0);
    EXPECT_DOUBLE_EQ(C(1, 0), 85.0);
    EXPECT_DOUBLE_EQ(C(1, 1), 55.0);
}

TEST(MatrixTest, Inversion) {
    Matrix<double, 2, 2> A = {4, 7, 2, 6};
    // Det = 24 - 14 = 10
    // Inv = 1/10 * [6 -7; -2 4] = [0.6 -0.7; -0.2 0.4]

    auto Inv = A.inverse();
    EXPECT_NEAR(Inv(0, 0), 0.6, 1e-9);
    EXPECT_NEAR(Inv(0, 1), -0.7, 1e-9);
    EXPECT_NEAR(Inv(1, 0), -0.2, 1e-9);
    EXPECT_NEAR(Inv(1, 1), 0.4, 1e-9);

    // Verify A * Inv = I
    auto I = A * Inv;
    EXPECT_NEAR(I(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(I(0, 1), 0.0, 1e-9);
}

// --- Kalman Filter Tests ---

class TestKalmanFilter : public ::testing::Test {
protected:
    // 2D State (Position, Velocity), 1D Measurement (Position)
    KalmanFilter<2, 1> kf;

    void SetUp() override {
        // Delta time
        double dt = 1.0;

        // Transition Matrix F: pos = pos + vel*dt
        Matrix<double, 2, 2> F = {1.0, dt, 0.0, 1.0};
        kf.setTransitionMatrix(F);

        // Measurement Matrix H: observing position only
        Matrix<double, 1, 2> H = {1.0, 0.0};
        kf.setMeasurementMatrix(H);

        // Process Noise Q
        Matrix<double, 2, 2> Q = {0.1, 0.0, 0.0, 0.1};
        kf.setProcessNoise(Q);

        // Measurement Noise R
        Matrix<double, 1, 1> R = {0.5};
        kf.setMeasurementNoise(R);

        // Initial State
        Vector<double, 2> x0 = {0.0, 0.0};
        Matrix<double, 2, 2> P0 = {10.0, 0.0, 0.0, 10.0};
        kf.init(x0, P0);
    }
};

TEST_F(TestKalmanFilter, Initialization) {
    auto x = kf.getState();
    EXPECT_DOUBLE_EQ(x(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(x(1, 0), 0.0);
}

TEST_F(TestKalmanFilter, Predict) {
    // Initial state is 0,0. Predict should stay 0,0 but Covariance should increase.
    kf.predict();

    auto x = kf.getState();
    EXPECT_DOUBLE_EQ(x(0, 0), 0.0);

    auto P = kf.getCovariance();
    // P = F*P0*F^T + Q
    // F = [1 1; 0 1], P0=[10 0; 0 10]
    // F*P0 = [10 10; 0 10]
    // (F*P0)*F^T = [10 10; 0 10] * [1 0; 1 1] = [20 10; 10 10]
    // + Q (0.1 diag) = [20.1 10.0; 10.0 10.1]

    EXPECT_NEAR(P(0, 0), 20.1, 1e-9);
    EXPECT_NEAR(P(1, 1), 10.1, 1e-9);
}

TEST_F(TestKalmanFilter, Update) {
    // Predict first
    kf.predict();

    // Measurement z = 10.0 (object moved to 10)
    Vector<double, 1> z = {10.0};

    kf.update(z);

    auto x = kf.getState();
    // Should have moved towards 10, but not all the way
    EXPECT_GT(x(0, 0), 0.0);
    EXPECT_LT(x(0, 0), 10.0);

    // Velocity should interpret this jump as positive velocity
    EXPECT_GT(x(1, 0), 0.0);

    // Covariance should decrease after measurement
    auto P = kf.getCovariance();
    EXPECT_LT(P(0, 0), 20.1);
}
