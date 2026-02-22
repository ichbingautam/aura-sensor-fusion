/**
 * @file test_data_association.cpp
 * @brief Unit tests for Data Association algorithms
 */

#include <aura/fusion/data_association.hpp>

#include <gtest/gtest.h>

using namespace aura::fusion;

TEST(DataAssociationTest, GlobalNearestNeighborGreedy) {
    // 3 Tracks, 4 Measurements
    std::vector<std::vector<double>> distance_matrix = {
        {1.0, 5.0, 8.0, 9.0}, {4.0, 2.0, 7.0, 9.0}, {6.0, 5.0, 3.0, 9.0}};

    double threshold = 5.0;

    auto associations = globalNearestNeighborGreedy(distance_matrix, threshold);

    // Expected associations:
    // T0 -> M0 (dist=1.0)
    // T1 -> M1 (dist=2.0)
    // T2 -> M2 (dist=3.0)
    // M3 is left unassociated because no track is close

    EXPECT_EQ(associations.size(), 3);

    bool found_t0_m0 = false;
    bool found_t1_m1 = false;
    bool found_t2_m2 = false;

    for (const auto& a : associations) {
        if (a.track_idx == 0 && a.measurement_idx == 0)
            found_t0_m0 = true;
        if (a.track_idx == 1 && a.measurement_idx == 1)
            found_t1_m1 = true;
        if (a.track_idx == 2 && a.measurement_idx == 2)
            found_t2_m2 = true;
    }

    EXPECT_TRUE(found_t0_m0);
    EXPECT_TRUE(found_t1_m1);
    EXPECT_TRUE(found_t2_m2);
}

TEST(DataAssociationTest, GNNThresholding) {
    // 2 Tracks, 2 Measurements
    std::vector<std::vector<double>> distance_matrix = {{10.0, 20.0}, {15.0, 12.0}};

    double threshold = 5.0;  // All distances > threshold

    auto associations = globalNearestNeighborGreedy(distance_matrix, threshold);

    EXPECT_EQ(associations.size(), 0);
}
