/**
 * @file test_pipeline.cpp
 * @brief Integration tests for fusion pipeline
 */

#include <aura/concurrency/thread_pool.hpp>
#include <aura/fusion/fusion_node.hpp>
#include <aura/sensors/sensor_base.hpp>

#include <gtest/gtest.h>

namespace aura::test {

class PipelineTest : public ::testing::Test {
protected:
    void SetUp() override { Logger::instance().setLevel(LogLevel::Off); }
};

TEST_F(PipelineTest, PassThroughNode) {
    FusionNodeConfig config;
    config.name = "passthrough";
    PassThroughNode node(config);

    EXPECT_TRUE(node.initialize());
    EXPECT_TRUE(node.start());
    EXPECT_EQ(node.state(), NodeState::Running);

    // Create test data
    std::vector<LidarPoint> points = {{1.0, 2.0, 3.0, 0.5}};
    auto input = std::make_unique<LidarData>(points, Timestamp::now(), 1, 1);

    auto output = node.process(std::move(input));
    EXPECT_NE(output, nullptr);

    node.stop();
    EXPECT_EQ(node.state(), NodeState::Stopped);
}

TEST_F(PipelineTest, NodeStats) {
    FusionNodeConfig config;
    config.name = "test_node";
    PassThroughNode node(config);
    node.initialize();
    node.start();

    for (int i = 0; i < 10; ++i) {
        std::vector<LidarPoint> points = {{1.0, 0.0, 0.0, 0.5}};
        auto input = std::make_unique<LidarData>(points, Timestamp::now(), 1, i);
        node.process(std::move(input));
    }

    auto stats = node.stats();
    EXPECT_EQ(stats.inputs_processed, 0);  // PassThrough doesn't track
}

TEST_F(PipelineTest, ConcurrentNodeAccess) {
    FusionNodeConfig config;
    config.name = "concurrent_test";
    config.input_buffer_size = 100;
    PassThroughNode node(config);
    node.initialize();
    node.start();

    ThreadPool pool(4);
    std::atomic<int> success_count{0};

    for (int i = 0; i < 50; ++i) {
        pool.execute(TaskPriority::Normal, [&node, &success_count, i]() {
            std::vector<LidarPoint> points = {{1.0, 0.0, 0.0, 0.5}};
            auto input = std::make_unique<LidarData>(points, Timestamp::now(), 1, i);
            node.submitInput(std::move(input));
            success_count.fetch_add(1);
        });
    }

    pool.waitAll();
    EXPECT_EQ(success_count.load(), 50);
}

}  // namespace aura::test
