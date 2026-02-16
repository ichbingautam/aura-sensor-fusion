/**
 * @file test_pipeline_coordinator.cpp
 * @brief Unit tests for PipelineCoordinator
 */

#include <aura/concurrency/thread_pool.hpp>
#include <aura/fusion/fusion_node.hpp>
#include <aura/fusion/pipeline_coordinator.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

using namespace aura;

// Mock node for testing
class MockFusionNode : public FusionNodeBase {
public:
    MockFusionNode(std::string name) : FusionNodeBase(FusionNodeConfig{.name = name}) {}

    bool initialize() override {
        initialized_ = true;
        setState(NodeState::Initialized);
        return true;
    }

    bool start() override {
        started_ = true;
        setState(NodeState::Running);
        return true;
    }

    void stop() override {
        stopped_ = true;
        setState(NodeState::Stopped);
    }

    OutputPtr process(InputPtr input) override {
        processed_count_++;
        // Return dummy output to trigger callback
        return std::make_unique<SensorData<int>>(1, Timestamp::now(), 0, 0);
    }

    std::atomic<bool> initialized_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stopped_{false};
    std::atomic<int> processed_count_{0};
};

class PipelineCoordinatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use a dedicated pool for testing
        pool_ = std::make_unique<ThreadPool>(2);
        coordinator_ = std::make_unique<PipelineCoordinator>(*pool_);
    }

    void TearDown() override {
        if (coordinator_) {
            coordinator_->stop();
        }
        if (pool_) {
            pool_->shutdown();
        }
    }

    std::unique_ptr<ThreadPool> pool_;
    std::unique_ptr<PipelineCoordinator> coordinator_;
};

TEST_F(PipelineCoordinatorTest, LifecycleManagement) {
    auto node1 = std::make_shared<MockFusionNode>("Node1");
    auto node2 = std::make_shared<MockFusionNode>("Node2");

    coordinator_->registerNode(node1);
    coordinator_->registerNode(node2);

    EXPECT_FALSE(node1->initialized_);
    EXPECT_FALSE(node1->started_);

    ASSERT_TRUE(coordinator_->start());

    // Give some time for thread to start (though start() calls node start synchronously)
    EXPECT_TRUE(node1->initialized_);
    EXPECT_TRUE(node1->started_);
    EXPECT_TRUE(node2->initialized_);
    EXPECT_TRUE(node2->started_);

    coordinator_->stop();

    EXPECT_TRUE(node1->stopped_);
    EXPECT_TRUE(node2->stopped_);
}

TEST_F(PipelineCoordinatorTest, DataProcessing) {
    auto node = std::make_shared<MockFusionNode>("WorkerNode");
    coordinator_->registerNode(node);

    ASSERT_TRUE(coordinator_->start());

    // Submit some data
    node->submitInput(std::make_unique<SensorData<int>>(42, Timestamp::now(), 0, 0));
    node->submitInput(std::make_unique<SensorData<int>>(43, Timestamp::now(), 0, 1));
    node->submitInput(std::make_unique<SensorData<int>>(44, Timestamp::now(), 0, 2));

    // Wait for processing
    int retries = 0;
    while (node->processed_count_ < 3 && retries++ < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(node->processed_count_, 3);
}

TEST_F(PipelineCoordinatorTest, ChainedProcessing) {
    auto node1 = std::make_shared<MockFusionNode>("Node1");
    auto node2 = std::make_shared<MockFusionNode>("Node2");

    // Link node1 -> node2
    node1->setOutputCallback(
        [node2](std::unique_ptr<SensorDataBase> output) { node2->submitInput(std::move(output)); });

    coordinator_->registerNode(node1);
    coordinator_->registerNode(node2);
    coordinator_->start();

    // Feed node1
    node1->submitInput(std::make_unique<SensorData<int>>(100, Timestamp::now(), 0, 0));
    node1->submitInput(std::make_unique<SensorData<int>>(101, Timestamp::now(), 0, 1));

    // Wait for node2 to process
    // Node1 processes -> callback -> Node2 submit -> Coordinator sees Node2 has input -> Node2
    // processes
    int retries = 0;
    while (node2->processed_count_ < 2 && retries++ < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(node1->processed_count_, 2);
    EXPECT_EQ(node2->processed_count_, 2);
}
