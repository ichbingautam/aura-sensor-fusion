/**
 * @file test_sensors.cpp
 * @brief Unit tests for sensor interfaces
 */

#include <aura/sensors/sensor_base.hpp>
#include <aura/sensors/sensor_data.hpp>
#include <aura/sensors/sensor_traits.hpp>

#include <gtest/gtest.h>

namespace aura::test {

class SensorTest : public ::testing::Test {};

TEST_F(SensorTest, SensorData_Construction) {
    std::vector<LidarPoint> points = {{1.0, 2.0, 3.0, 0.5}};
    auto timestamp = Timestamp::now();

    SensorData<std::vector<LidarPoint>> data(points, timestamp, 1, 100, "lidar");

    EXPECT_EQ(data.data().size(), 1);
    EXPECT_EQ(data.sensorId(), 1);
    EXPECT_TRUE(data.isValid());
}

TEST_F(SensorTest, SensorConfig_Parameters) {
    SensorConfig config;
    config.name = "test_sensor";
    config.parameters["key"] = "value";

    EXPECT_EQ(config.getParam("key"), "value");
    EXPECT_EQ(config.getParam("missing", "default"), "default");
}

TEST_F(SensorTest, SensorTraits_LidarSensor) {
    using Traits = SensorTraits<LidarSensor>;
    EXPECT_EQ(Traits::sensor_type, SensorType::Lidar);
    EXPECT_TRUE(Traits::is_continuous);
}

TEST_F(SensorTest, Concepts_ValidSensor) {
    static_assert(ValidSensor<LidarSensor>);
    static_assert(ValidSensor<RadarSensor>);
    static_assert(PointCloudSensor<LidarSensor>);
    static_assert(!ContinuousSensor<GpsSensor>);
}

}  // namespace aura::test
