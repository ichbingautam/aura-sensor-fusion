/**
 * @file test_memory_pool.cpp
 * @brief Unit tests for memory pool
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#include <aura/core/memory_pool.hpp>

#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace aura::test {

class MemoryPoolTest : public ::testing::Test {};

TEST_F(MemoryPoolTest, BasicAllocation) {
    MemoryPool<float> pool(1024, 4);

    auto block = pool.acquire();
    EXPECT_TRUE(block);
    EXPECT_EQ(block.size(), 1024);
    EXPECT_NE(block.data(), nullptr);
}

TEST_F(MemoryPoolTest, MultipleAllocations) {
    MemoryPool<int> pool(100, 4);

    std::vector<PooledBlock<int>> blocks;
    for (int i = 0; i < 4; ++i) {
        auto block = pool.acquire();
        EXPECT_TRUE(block);
        blocks.push_back(std::move(block));
    }

    EXPECT_EQ(pool.freeBlocks(), 0);
}

TEST_F(MemoryPoolTest, BlockReuse) {
    MemoryPool<int> pool(100, 2);

    void* first_ptr;
    {
        auto block = pool.acquire();
        first_ptr = block.data();
        // Block is released here
    }

    // Should get the same block back
    auto block2 = pool.acquire();
    EXPECT_EQ(block2.data(), first_ptr);
}

TEST_F(MemoryPoolTest, BlockDataAccess) {
    MemoryPool<int> pool(10, 1);

    auto block = pool.acquire();

    // Write data
    for (std::size_t i = 0; i < block.size(); ++i) {
        block[i] = static_cast<int>(i * 2);
    }

    // Verify data
    for (std::size_t i = 0; i < block.size(); ++i) {
        EXPECT_EQ(block[i], static_cast<int>(i * 2));
    }
}

TEST_F(MemoryPoolTest, SpanAccess) {
    MemoryPool<double> pool(100, 1);

    auto block = pool.acquire();
    auto span = block.span();

    EXPECT_EQ(span.size(), 100);
    span[50] = 3.14159;
    EXPECT_DOUBLE_EQ(block[50], 3.14159);
}

TEST_F(MemoryPoolTest, Stats) {
    MemoryPool<int> pool(100, 4);

    auto stats = pool.stats();
    EXPECT_EQ(stats.total_blocks, 4);
    EXPECT_EQ(stats.free_blocks, 4);
    EXPECT_EQ(stats.used_blocks, 0);

    auto block = pool.acquire();
    stats = pool.stats();
    EXPECT_EQ(stats.used_blocks, 1);
    EXPECT_EQ(stats.free_blocks, 3);
}

TEST_F(MemoryPoolTest, DynamicGrowth) {
    MemoryPool<int> pool(100, 2, 0);  // Unlimited max

    // Acquire more than initial
    std::vector<PooledBlock<int>> blocks;
    for (int i = 0; i < 10; ++i) {
        blocks.push_back(pool.acquire());
    }

    EXPECT_EQ(pool.totalBlocks(), 10);
}

TEST_F(MemoryPoolTest, MaxBlocksLimit) {
    MemoryPool<int> pool(100, 2, 3);  // Max 3 blocks

    std::vector<PooledBlock<int>> blocks;
    for (int i = 0; i < 3; ++i) {
        blocks.push_back(pool.acquire());
    }

    // Fourth should fail
    EXPECT_THROW(pool.acquire(), std::bad_alloc);
}

TEST_F(MemoryPoolTest, TryAcquire) {
    MemoryPool<int> pool(100, 1, 1);

    auto block1 = pool.tryAcquire();
    EXPECT_TRUE(block1.has_value());

    auto block2 = pool.tryAcquire();
    EXPECT_FALSE(block2.has_value());
}

TEST_F(MemoryPoolTest, MoveSemantics) {
    MemoryPool<int> pool(100, 1);

    auto block1 = pool.acquire();
    auto* ptr = block1.data();

    // Move construct
    PooledBlock<int> block2 = std::move(block1);
    EXPECT_EQ(block2.data(), ptr);
    EXPECT_FALSE(block1);  // NOLINT - testing moved-from state

    // Move assign
    PooledBlock<int> block3;
    block3 = std::move(block2);
    EXPECT_EQ(block3.data(), ptr);
}

TEST_F(MemoryPoolTest, ConcurrentAccess) {
    MemoryPool<int> pool(100, 8);
    constexpr int kNumThreads = 4;
    constexpr int kIterations = 100;

    std::atomic<int> success_count{0};

    std::vector<std::thread> threads;
    for (int t = 0; t < kNumThreads; ++t) {
        threads.emplace_back([&pool, &success_count]() {
            for (int i = 0; i < kIterations; ++i) {
                if (auto block = pool.tryAcquire()) {
                    // Simulate some work
                    std::this_thread::yield();
                    success_count.fetch_add(1);
                }
                // Block is released here
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    // All iterations should succeed since we release blocks
    EXPECT_EQ(success_count.load(), kNumThreads * kIterations);
}

// AlignedDataBuffer tests
class AlignedDataBufferTest : public ::testing::Test {};

TEST_F(AlignedDataBufferTest, BasicConstruction) {
    AlignedDataBuffer<float> buffer(100);
    EXPECT_EQ(buffer.size(), 100);
    EXPECT_NE(buffer.data(), nullptr);
}

TEST_F(AlignedDataBufferTest, DataAccess) {
    AlignedDataBuffer<int> buffer(10);

    for (std::size_t i = 0; i < buffer.size(); ++i) {
        buffer[i] = static_cast<int>(i * 3);
    }

    for (std::size_t i = 0; i < buffer.size(); ++i) {
        EXPECT_EQ(buffer[i], static_cast<int>(i * 3));
    }
}

TEST_F(AlignedDataBufferTest, SpanView) {
    AlignedDataBuffer<double> buffer(50);
    auto span = buffer.span();

    EXPECT_EQ(span.size(), 50);
    span[25] = 2.718;
    EXPECT_DOUBLE_EQ(buffer[25], 2.718);
}

}  // namespace aura::test
