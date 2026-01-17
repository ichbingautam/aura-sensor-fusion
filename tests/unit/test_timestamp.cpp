/**
 * @file test_timestamp.cpp
 * @brief Unit tests for timestamp utilities
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#include <aura/core/timestamp.hpp>

#include <thread>

#include <gtest/gtest.h>

namespace aura::test {

class TimestampTest : public ::testing::Test {};

TEST_F(TimestampTest, DefaultConstruction) {
    Timestamp ts;
    EXPECT_TRUE(ts.isZero());
    EXPECT_EQ(ts.nanoseconds(), 0);
}

TEST_F(TimestampTest, FromNanoseconds) {
    auto ts = Timestamp::fromNanoseconds(1'000'000'000);
    EXPECT_EQ(ts.nanoseconds(), 1'000'000'000);
    EXPECT_EQ(ts.microseconds(), 1'000'000);
    EXPECT_EQ(ts.milliseconds(), 1'000);
    EXPECT_DOUBLE_EQ(ts.seconds(), 1.0);
}

TEST_F(TimestampTest, FromSeconds) {
    auto ts = Timestamp::fromSeconds(2.5);
    EXPECT_NEAR(ts.seconds(), 2.5, 1e-9);
    EXPECT_EQ(ts.nanoseconds(), 2'500'000'000);
}

TEST_F(TimestampTest, FromMilliseconds) {
    auto ts = Timestamp::fromMilliseconds(500);
    EXPECT_EQ(ts.milliseconds(), 500);
    EXPECT_DOUBLE_EQ(ts.seconds(), 0.5);
}

TEST_F(TimestampTest, FromMicroseconds) {
    auto ts = Timestamp::fromMicroseconds(1000);
    EXPECT_EQ(ts.microseconds(), 1000);
    EXPECT_EQ(ts.milliseconds(), 1);
}

TEST_F(TimestampTest, Comparison) {
    auto ts1 = Timestamp::fromSeconds(1.0);
    auto ts2 = Timestamp::fromSeconds(2.0);
    auto ts3 = Timestamp::fromSeconds(1.0);

    EXPECT_LT(ts1, ts2);
    EXPECT_GT(ts2, ts1);
    EXPECT_EQ(ts1, ts3);
    EXPECT_LE(ts1, ts3);
    EXPECT_GE(ts1, ts3);
}

TEST_F(TimestampTest, Arithmetic) {
    auto ts1 = Timestamp::fromSeconds(1.0);
    auto ts2 = Timestamp::fromSeconds(0.5);

    auto sum = ts1 + ts2;
    EXPECT_DOUBLE_EQ(sum.seconds(), 1.5);

    auto diff = ts1 - ts2;
    EXPECT_DOUBLE_EQ(diff.seconds(), 0.5);
}

TEST_F(TimestampTest, CompoundArithmetic) {
    auto ts = Timestamp::fromSeconds(1.0);
    ts += Timestamp::fromSeconds(0.5);
    EXPECT_DOUBLE_EQ(ts.seconds(), 1.5);

    ts -= Timestamp::fromSeconds(0.25);
    EXPECT_DOUBLE_EQ(ts.seconds(), 1.25);
}

TEST_F(TimestampTest, AbsDiff) {
    auto ts1 = Timestamp::fromSeconds(3.0);
    auto ts2 = Timestamp::fromSeconds(1.0);

    auto diff = ts1.absDiff(ts2);
    EXPECT_DOUBLE_EQ(diff.seconds(), 2.0);

    // Same result in reverse order
    diff = ts2.absDiff(ts1);
    EXPECT_DOUBLE_EQ(diff.seconds(), 2.0);
}

TEST_F(TimestampTest, IsWithin) {
    auto ts1 = Timestamp::fromSeconds(1.0);
    auto ts2 = Timestamp::fromSeconds(1.05);
    auto tolerance = std::chrono::milliseconds(100);

    EXPECT_TRUE(ts1.isWithin(ts2, tolerance));
    EXPECT_FALSE(ts1.isWithin(Timestamp::fromSeconds(2.0), tolerance));
}

TEST_F(TimestampTest, Now) {
    auto ts1 = Timestamp::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto ts2 = Timestamp::now();

    EXPECT_GT(ts2, ts1);
    EXPECT_GE((ts2 - ts1).milliseconds(), 10);
}

TEST_F(TimestampTest, MinMax) {
    EXPECT_GT(Timestamp::max(), Timestamp::min());
    EXPECT_GT(Timestamp::max(), Timestamp::now());
}

TEST_F(TimestampTest, ToDuration) {
    auto ts = Timestamp::fromMilliseconds(500);
    auto duration = ts.toDuration();
    EXPECT_EQ(duration.count(), 500'000'000);  // nanoseconds
}

// TimeInterval tests
TEST_F(TimestampTest, TimeInterval_FromSeconds) {
    auto interval = TimeInterval::fromSeconds(1.5);
    EXPECT_DOUBLE_EQ(interval.seconds(), 1.5);
    EXPECT_EQ(interval.milliseconds(), 1500);
}

TEST_F(TimestampTest, TimeInterval_FromMilliseconds) {
    auto interval = TimeInterval::fromMilliseconds(250);
    EXPECT_EQ(interval.milliseconds(), 250);
    EXPECT_DOUBLE_EQ(interval.seconds(), 0.25);
}

// ScopedTimer tests
TEST_F(TimestampTest, ScopedTimer_Elapsed) {
    ScopedTimer timer;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_GE(timer.elapsedMs(), 50);
    EXPECT_GT(timer.elapsedUs(), 50000);
    EXPECT_GT(timer.elapsedNs(), 50000000);
}

TEST_F(TimestampTest, ScopedTimer_Reset) {
    ScopedTimer timer;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    timer.reset();
    EXPECT_LT(timer.elapsedMs(), 10);
}

TEST_F(TimestampTest, ScopedTimer_ElapsedSeconds) {
    ScopedTimer timer;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_GE(timer.elapsedSeconds(), 0.1);
}

}  // namespace aura::test
