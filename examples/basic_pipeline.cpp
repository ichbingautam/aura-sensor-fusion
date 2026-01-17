/**
 * @file basic_pipeline.cpp
 * @brief Basic example of using AURA fusion pipeline
 *
 * This example demonstrates:
 * - Creating sensor data
 * - Using the thread pool for parallel processing
 * - Measuring performance with profiler
 */

#include <aura/aura.hpp>

#include <iostream>
#include <random>
#include <vector>

using namespace aura;

// Generate synthetic LiDAR point cloud
std::vector<LidarPoint> generatePointCloud(std::size_t num_points) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<Float> dist_xy(-50.0, 50.0);
    std::uniform_real_distribution<Float> dist_z(-2.0, 5.0);
    std::uniform_real_distribution<Float> dist_intensity(0.0, 1.0);

    std::vector<LidarPoint> points;
    points.reserve(num_points);

    for (std::size_t i = 0; i < num_points; ++i) {
        points.push_back({dist_xy(gen), dist_xy(gen), dist_z(gen), dist_intensity(gen),
                          static_cast<std::uint16_t>(i % 64), 0.0f});
    }

    return points;
}

// Simple point cloud filter
std::vector<LidarPoint> filterGroundPoints(const std::vector<LidarPoint>& points,
                                           Float z_threshold = -1.5) {
    std::vector<LidarPoint> filtered;
    filtered.reserve(points.size());

    for (const auto& p : points) {
        if (p.z > z_threshold) {
            filtered.push_back(p);
        }
    }

    return filtered;
}

int main() {
    std::cout << "AURA Sensor Fusion - Basic Pipeline Example\n";
    std::cout << "Version: " << version() << "\n\n";

    // Configure logging
    Logger::instance().setLevel(LogLevel::Info);

    // Create thread pool
    ThreadPool pool(4);
    AURA_LOG_INFO("Thread pool started with {} threads", pool.numThreads());

    // Generate test data
    constexpr std::size_t kNumPoints = 100000;
    AURA_LOG_INFO("Generating {} LiDAR points...", kNumPoints);

    auto points = generatePointCloud(kNumPoints);
    AURA_LOG_INFO("Generated {} points", points.size());

    // Process data using thread pool
    AURA_LOG_INFO("Processing point cloud...");

    ScopedTimer timer;

    // Split into chunks for parallel processing
    constexpr std::size_t kChunkSize = 10000;
    std::vector<std::future<std::vector<LidarPoint>>> futures;

    for (std::size_t i = 0; i < points.size(); i += kChunkSize) {
        std::size_t end = std::min(i + kChunkSize, points.size());
        std::vector<LidarPoint> chunk(points.begin() + static_cast<std::ptrdiff_t>(i),
                                      points.begin() + static_cast<std::ptrdiff_t>(end));

        futures.push_back(
            pool.submit([chunk = std::move(chunk)]() { return filterGroundPoints(chunk); }));
    }

    // Collect results
    std::vector<LidarPoint> filtered_points;
    for (auto& f : futures) {
        auto result = f.get();
        filtered_points.insert(filtered_points.end(), result.begin(), result.end());
    }

    auto elapsed_ms = timer.elapsedMs();

    AURA_LOG_INFO("Filtered {} -> {} points in {} ms", points.size(), filtered_points.size(),
                  elapsed_ms);

    // Calculate throughput
    double throughput = static_cast<double>(points.size()) / (elapsed_ms / 1000.0);
    AURA_LOG_INFO("Throughput: {:.2f} points/second", throughput);

    // Print profiler report
    std::cout << "\n" << Profiler::instance().report() << std::endl;

    return 0;
}
