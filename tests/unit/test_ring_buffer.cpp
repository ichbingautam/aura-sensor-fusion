/**
 * @file test_ring_buffer.cpp
 * @brief Unit tests for lock-free ring buffer
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#include <aura/concurrency/lock_free_ring_buffer.hpp>

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace aura::test {

// Test fixture for ring buffer tests
class RingBufferTest : public ::testing::Test {
protected:
    static constexpr std::size_t kBufferSize = 64;
    using IntBuffer = LockFreeRingBuffer<int, kBufferSize>;
};

TEST_F(RingBufferTest, DefaultConstructor) {
    IntBuffer buffer;
    EXPECT_TRUE(buffer.empty());
    EXPECT_FALSE(buffer.full());
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_EQ(buffer.capacity(), kBufferSize - 1);
}

TEST_F(RingBufferTest, PushAndPop) {
    IntBuffer buffer;

    // Push a value
    EXPECT_TRUE(buffer.tryPush(42));
    EXPECT_FALSE(buffer.empty());
    EXPECT_EQ(buffer.size(), 1);

    // Pop the value
    auto result = buffer.tryPop();
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(*result, 42);
    EXPECT_TRUE(buffer.empty());
}

TEST_F(RingBufferTest, MultipleValues) {
    IntBuffer buffer;

    // Push multiple values
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(buffer.tryPush(i));
    }
    EXPECT_EQ(buffer.size(), 10);

    // Pop all values
    for (int i = 0; i < 10; ++i) {
        auto result = buffer.tryPop();
        EXPECT_TRUE(result.has_value());
        EXPECT_EQ(*result, i);
    }
    EXPECT_TRUE(buffer.empty());
}

TEST_F(RingBufferTest, FullBuffer) {
    IntBuffer buffer;

    // Fill the buffer
    for (std::size_t i = 0; i < buffer.capacity(); ++i) {
        EXPECT_TRUE(buffer.tryPush(static_cast<int>(i)));
    }
    EXPECT_TRUE(buffer.full());

    // Try to push to full buffer
    EXPECT_FALSE(buffer.tryPush(999));
}

TEST_F(RingBufferTest, EmptyBuffer) {
    IntBuffer buffer;

    // Try to pop from empty buffer
    auto result = buffer.tryPop();
    EXPECT_FALSE(result.has_value());
}

TEST_F(RingBufferTest, Peek) {
    IntBuffer buffer;

    // Empty buffer peek
    EXPECT_EQ(buffer.peek(), nullptr);

    // Push and peek
    buffer.tryPush(42);
    const int* peek_result = buffer.peek();
    EXPECT_NE(peek_result, nullptr);
    EXPECT_EQ(*peek_result, 42);

    // Peek doesn't remove element
    EXPECT_EQ(buffer.size(), 1);
}

TEST_F(RingBufferTest, Clear) {
    IntBuffer buffer;

    for (int i = 0; i < 10; ++i) {
        buffer.tryPush(i);
    }
    EXPECT_EQ(buffer.size(), 10);

    buffer.clear();
    EXPECT_TRUE(buffer.empty());
}

TEST_F(RingBufferTest, TryEmplace) {
    LockFreeRingBuffer<std::pair<int, int>, 16> buffer;

    EXPECT_TRUE(buffer.tryEmplace(1, 2));
    EXPECT_EQ(buffer.size(), 1);

    auto result = buffer.tryPop();
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->first, 1);
    EXPECT_EQ(result->second, 2);
}

TEST_F(RingBufferTest, WrapAround) {
    IntBuffer buffer;

    // Fill and drain several times to test wrap-around
    for (int round = 0; round < 5; ++round) {
        // Fill half
        for (int i = 0; i < 30; ++i) {
            EXPECT_TRUE(buffer.tryPush(i + round * 100));
        }

        // Drain all
        for (int i = 0; i < 30; ++i) {
            auto result = buffer.tryPop();
            EXPECT_TRUE(result.has_value());
            EXPECT_EQ(*result, i + round * 100);
        }
    }
}

TEST_F(RingBufferTest, SingleProducerSingleConsumer) {
    IntBuffer buffer;
    constexpr int kNumItems = 10000;
    std::atomic<int> consumed_sum{0};

    // Producer thread
    std::thread producer([&buffer]() {
        for (int i = 0; i < kNumItems; ++i) {
            while (!buffer.tryPush(i)) {
                std::this_thread::yield();
            }
        }
    });

    // Consumer thread
    std::thread consumer([&buffer, &consumed_sum]() {
        int count = 0;
        while (count < kNumItems) {
            if (auto result = buffer.tryPop()) {
                consumed_sum.fetch_add(*result, std::memory_order_relaxed);
                ++count;
            } else {
                std::this_thread::yield();
            }
        }
    });

    producer.join();
    consumer.join();

    // Verify sum: 0 + 1 + 2 + ... + (n-1) = n*(n-1)/2
    int expected_sum = (kNumItems * (kNumItems - 1)) / 2;
    EXPECT_EQ(consumed_sum.load(), expected_sum);
}

// MPMC Ring Buffer Tests
class MPMCRingBufferTest : public ::testing::Test {
protected:
    using IntBuffer = MPMCRingBuffer<int>;
};

TEST_F(MPMCRingBufferTest, BasicOperations) {
    IntBuffer buffer(64);

    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.capacity(), 64);

    EXPECT_TRUE(buffer.tryPush(42));
    EXPECT_FALSE(buffer.empty());

    auto result = buffer.tryPop();
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(*result, 42);
}

TEST_F(MPMCRingBufferTest, MultipleProducersMultipleConsumers) {
    IntBuffer buffer(1024);
    constexpr int kNumProducers = 4;
    constexpr int kNumConsumers = 4;
    constexpr int kItemsPerProducer = 1000;

    std::atomic<int> produced_count{0};
    std::atomic<int> consumed_count{0};
    std::atomic<bool> done{false};

    std::vector<std::thread> producers;
    std::vector<std::thread> consumers;

    // Start producers
    for (int p = 0; p < kNumProducers; ++p) {
        producers.emplace_back([&buffer, &produced_count, p]() {
            for (int i = 0; i < kItemsPerProducer; ++i) {
                int value = p * kItemsPerProducer + i;
                while (!buffer.tryPush(std::move(value))) {
                    std::this_thread::yield();
                }
                produced_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    // Start consumers
    for (int c = 0; c < kNumConsumers; ++c) {
        consumers.emplace_back([&buffer, &consumed_count, &done]() {
            while (!done.load(std::memory_order_acquire) || !buffer.empty()) {
                if (buffer.tryPop()) {
                    consumed_count.fetch_add(1, std::memory_order_relaxed);
                } else {
                    std::this_thread::yield();
                }
            }
        });
    }

    // Wait for producers to finish
    for (auto& t : producers) {
        t.join();
    }

    done.store(true, std::memory_order_release);

    // Wait for consumers to finish
    for (auto& t : consumers) {
        t.join();
    }

    EXPECT_EQ(produced_count.load(), kNumProducers * kItemsPerProducer);
    EXPECT_EQ(consumed_count.load(), kNumProducers * kItemsPerProducer);
}

}  // namespace aura::test
