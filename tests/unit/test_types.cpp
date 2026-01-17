/**
 * @file test_types.cpp
 * @brief Unit tests for core types
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#include <aura/core/types.hpp>

#include <cmath>

#include <gtest/gtest.h>

namespace aura::test {

class TypesTest : public ::testing::Test {};

// Point3D tests
TEST_F(TypesTest, Point3D_DefaultConstruction) {
    Point3D p;
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
    EXPECT_DOUBLE_EQ(p.z, 0.0);
}

TEST_F(TypesTest, Point3D_Construction) {
    Point3D p(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(p.x, 1.0);
    EXPECT_DOUBLE_EQ(p.y, 2.0);
    EXPECT_DOUBLE_EQ(p.z, 3.0);
}

TEST_F(TypesTest, Point3D_Distance) {
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(3.0, 4.0, 0.0);

    EXPECT_DOUBLE_EQ(p1.distanceTo(p2), 5.0);
    EXPECT_DOUBLE_EQ(p1.distanceSquaredTo(p2), 25.0);
}

TEST_F(TypesTest, Point3D_Arithmetic) {
    Point3D p1(1.0, 2.0, 3.0);
    Point3D p2(4.0, 5.0, 6.0);

    auto sum = p1 + p2;
    EXPECT_DOUBLE_EQ(sum.x, 5.0);
    EXPECT_DOUBLE_EQ(sum.y, 7.0);
    EXPECT_DOUBLE_EQ(sum.z, 9.0);

    auto diff = p2 - p1;
    EXPECT_DOUBLE_EQ(diff.x, 3.0);
    EXPECT_DOUBLE_EQ(diff.y, 3.0);
    EXPECT_DOUBLE_EQ(diff.z, 3.0);

    auto scaled = p1 * 2.0;
    EXPECT_DOUBLE_EQ(scaled.x, 2.0);
    EXPECT_DOUBLE_EQ(scaled.y, 4.0);
    EXPECT_DOUBLE_EQ(scaled.z, 6.0);
}

// Vector3D tests
TEST_F(TypesTest, Vector3D_Magnitude) {
    Vector3D v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.magnitude(), 5.0);
    EXPECT_DOUBLE_EQ(v.magnitudeSquared(), 25.0);
}

TEST_F(TypesTest, Vector3D_Normalized) {
    Vector3D v(3.0, 0.0, 0.0);
    auto n = v.normalized();
    EXPECT_DOUBLE_EQ(n.x, 1.0);
    EXPECT_DOUBLE_EQ(n.y, 0.0);
    EXPECT_DOUBLE_EQ(n.z, 0.0);
}

TEST_F(TypesTest, Vector3D_DotProduct) {
    Vector3D v1(1.0, 0.0, 0.0);
    Vector3D v2(0.0, 1.0, 0.0);
    EXPECT_DOUBLE_EQ(v1.dot(v2), 0.0);  // Perpendicular

    Vector3D v3(1.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(v1.dot(v3), 1.0);  // Parallel
}

TEST_F(TypesTest, Vector3D_CrossProduct) {
    Vector3D v1(1.0, 0.0, 0.0);
    Vector3D v2(0.0, 1.0, 0.0);
    auto cross = v1.cross(v2);
    EXPECT_DOUBLE_EQ(cross.x, 0.0);
    EXPECT_DOUBLE_EQ(cross.y, 0.0);
    EXPECT_DOUBLE_EQ(cross.z, 1.0);
}

// Quaternion tests
TEST_F(TypesTest, Quaternion_Identity) {
    auto q = Quaternion::identity();
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
}

TEST_F(TypesTest, Quaternion_Normalized) {
    Quaternion q(2.0, 0.0, 0.0, 0.0);
    auto n = q.normalized();
    EXPECT_DOUBLE_EQ(n.w, 1.0);
}

TEST_F(TypesTest, Quaternion_Conjugate) {
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    auto c = q.conjugate();
    EXPECT_DOUBLE_EQ(c.w, 1.0);
    EXPECT_DOUBLE_EQ(c.x, -2.0);
    EXPECT_DOUBLE_EQ(c.y, -3.0);
    EXPECT_DOUBLE_EQ(c.z, -4.0);
}

TEST_F(TypesTest, Quaternion_RotatePoint) {
    // 90-degree rotation around Z axis
    Quaternion q(std::cos(M_PI / 4), 0.0, 0.0, std::sin(M_PI / 4));
    Point3D p(1.0, 0.0, 0.0);

    auto rotated = q.rotate(p);
    EXPECT_NEAR(rotated.x, 0.0, 1e-10);
    EXPECT_NEAR(rotated.y, 1.0, 1e-10);
    EXPECT_NEAR(rotated.z, 0.0, 1e-10);
}

// BoundingBox3D tests
TEST_F(TypesTest, BoundingBox3D_Volume) {
    BoundingBox3D box;
    box.length = 2.0;
    box.width = 3.0;
    box.height = 4.0;
    EXPECT_DOUBLE_EQ(box.volume(), 24.0);
}

TEST_F(TypesTest, BoundingBox3D_Contains) {
    BoundingBox3D box;
    box.center = {0.0, 0.0, 0.0};
    box.length = 2.0;
    box.width = 2.0;
    box.height = 2.0;
    box.yaw = 0.0;

    EXPECT_TRUE(box.contains({0.0, 0.0, 0.0}));
    EXPECT_TRUE(box.contains({0.5, 0.5, 0.5}));
    EXPECT_FALSE(box.contains({2.0, 0.0, 0.0}));
}

TEST_F(TypesTest, BoundingBox3D_Corners) {
    BoundingBox3D box;
    box.center = {0.0, 0.0, 0.0};
    box.length = 2.0;
    box.width = 2.0;
    box.height = 2.0;
    box.yaw = 0.0;

    auto corners = box.corners();
    EXPECT_EQ(corners.size(), 8);

    // Check one corner
    bool found_corner = false;
    for (const auto& c : corners) {
        if (std::abs(c.x - 1.0) < 1e-10 && std::abs(c.y - 1.0) < 1e-10 &&
            std::abs(c.z - 1.0) < 1e-10) {
            found_corner = true;
            break;
        }
    }
    EXPECT_TRUE(found_corner);
}

// LidarPoint tests
TEST_F(TypesTest, LidarPoint_ToPoint3D) {
    LidarPoint lp;
    lp.x = 1.0;
    lp.y = 2.0;
    lp.z = 3.0;
    lp.intensity = 0.5;

    auto p = lp.toPoint3D();
    EXPECT_DOUBLE_EQ(p.x, 1.0);
    EXPECT_DOUBLE_EQ(p.y, 2.0);
    EXPECT_DOUBLE_EQ(p.z, 3.0);
}

// RadarPoint tests
TEST_F(TypesTest, RadarPoint_ToPoint3D) {
    RadarPoint rp;
    rp.range = 10.0;
    rp.azimuth = 0.0;
    rp.elevation = 0.0;

    auto p = rp.toPoint3D();
    EXPECT_NEAR(p.x, 10.0, 1e-10);
    EXPECT_NEAR(p.y, 0.0, 1e-10);
    EXPECT_NEAR(p.z, 0.0, 1e-10);
}

// ObjectClass tests
TEST_F(TypesTest, ObjectClass_ToString) {
    EXPECT_STREQ(toString(ObjectClass::Car), "Car");
    EXPECT_STREQ(toString(ObjectClass::Pedestrian), "Pedestrian");
    EXPECT_STREQ(toString(ObjectClass::Unknown), "Unknown");
}

// Pose3D tests
TEST_F(TypesTest, Pose3D_Identity) {
    auto pose = Pose3D::identity();
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
}

TEST_F(TypesTest, Pose3D_TransformPoint) {
    Pose3D pose;
    pose.position = {10.0, 0.0, 0.0};
    pose.orientation = Quaternion::identity();

    Point3D local{1.0, 0.0, 0.0};
    auto world = pose.transformPoint(local);

    EXPECT_DOUBLE_EQ(world.x, 11.0);
    EXPECT_DOUBLE_EQ(world.y, 0.0);
    EXPECT_DOUBLE_EQ(world.z, 0.0);
}

}  // namespace aura::test
