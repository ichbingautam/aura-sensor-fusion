/**
 * @file sensor_simulation.cpp
 * @brief Example simulating multiple sensor streams
 */

#include <aura/aura.hpp>

#include <iostream>
#include <random>

using namespace aura;

int main() {
    std::cout << "AURA - Sensor Simulation Example\n\n";

    Logger::instance().setLevel(LogLevel::Info);

    // Create lock-free buffers for each sensor
    LockFreeRingBuffer<LidarData, 64> lidar_buffer;
    LockFreeRingBuffer<RadarData, 64> radar_buffer;

    // Simulate sensor data generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<Float> dist(-10.0, 10.0);

    // Generate LiDAR data
    AURA_LOG_INFO("Generating LiDAR data...");
    for (int frame = 0; frame < 10; ++frame) {
        std::vector<LidarPoint> points;
        for (int i = 0; i < 1000; ++i) {
            points.push_back({dist(gen), dist(gen), dist(gen), 0.5f, 0, 0.0f});
        }

        LidarData data(std::move(points), Timestamp::now(), 1, frame, "lidar");
        lidar_buffer.tryPush(std::move(data));
    }

    // Generate Radar data
    AURA_LOG_INFO("Generating Radar data...");
    for (int frame = 0; frame < 10; ++frame) {
        std::vector<RadarPoint> points;
        for (int i = 0; i < 50; ++i) {
            points.push_back({10.0 + dist(gen), dist(gen) * 0.1, 0.0, 5.0, -10.0, 20.0});
        }

        RadarData data(std::move(points), Timestamp::now(), 2, frame, "radar");
        radar_buffer.tryPush(std::move(data));
    }

    // Process data
    AURA_LOG_INFO("Processing sensor data...");
    int lidar_frames = 0, radar_frames = 0;

    while (auto lidar_data = lidar_buffer.tryPop()) {
        ++lidar_frames;
        AURA_LOG_DEBUG("Processed LiDAR frame {} with {} points", lidar_data->sequenceNumber(),
                       lidar_data->data().size());
    }

    while (auto radar_data = radar_buffer.tryPop()) {
        ++radar_frames;
        AURA_LOG_DEBUG("Processed Radar frame {} with {} points", radar_data->sequenceNumber(),
                       radar_data->data().size());
    }

    AURA_LOG_INFO("Processed {} LiDAR frames and {} Radar frames", lidar_frames, radar_frames);

    return 0;
}
