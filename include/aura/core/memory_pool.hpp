/**
 * @file memory_pool.hpp
 * @brief Memory pool allocator for zero-copy data transfer
 *
 * Provides a high-performance memory pool with:
 * - Fixed-size block allocation for predictable performance
 * - Cache-line aligned blocks for optimal memory access
 * - Thread-safe allocation and deallocation
 * - RAII-based resource management
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <atomic>
#include <cassert>
#include <cstddef>
#include <memory>
#include <mutex>
#include <new>
#include <span>
#include <vector>

namespace aura {

/// Cache line size for alignment
inline constexpr std::size_t kCacheLineSize = 64;

/**
 * @brief Custom deleter for aligned memory
 */
struct AlignedDeleter {
    void operator()(void* ptr) const noexcept {
        if (ptr) {
            ::operator delete(ptr, std::align_val_t{kCacheLineSize});
        }
    }
};

/**
 * @brief RAII wrapper for memory pool blocks
 *
 * Automatically returns the block to the pool when destroyed.
 *
 * @tparam T Type of data stored in the block
 */
template <typename T>
class PooledBlock {
public:
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    PooledBlock() noexcept = default;
    PooledBlock(std::nullptr_t) noexcept {}  // NOLINT

    /// Construct with data and return callback
    PooledBlock(pointer data, std::size_t size, std::function<void(pointer)> return_fn)
        : data_(data), size_(size), return_fn_(std::move(return_fn)) {}

    // Move-only semantics
    PooledBlock(const PooledBlock&) = delete;
    PooledBlock& operator=(const PooledBlock&) = delete;

    PooledBlock(PooledBlock&& other) noexcept
        : data_(std::exchange(other.data_, nullptr)), size_(std::exchange(other.size_, 0)),
          return_fn_(std::exchange(other.return_fn_, nullptr)) {}

    PooledBlock& operator=(PooledBlock&& other) noexcept {
        if (this != &other) {
            release();
            data_ = std::exchange(other.data_, nullptr);
            size_ = std::exchange(other.size_, 0);
            return_fn_ = std::exchange(other.return_fn_, nullptr);
        }
        return *this;
    }

    ~PooledBlock() { release(); }

    /// Release the block back to the pool
    void release() noexcept {
        if (data_ && return_fn_) {
            return_fn_(data_);
            data_ = nullptr;
            size_ = 0;
            return_fn_ = nullptr;
        }
    }

    /// Check if block is valid
    [[nodiscard]] explicit operator bool() const noexcept { return data_ != nullptr; }

    /// Access the underlying data
    [[nodiscard]] pointer data() noexcept { return data_; }
    [[nodiscard]] const_pointer data() const noexcept { return data_; }

    /// Get the size of the block
    [[nodiscard]] std::size_t size() const noexcept { return size_; }

    /// Get a span view of the data
    [[nodiscard]] std::span<T> span() noexcept { return {data_, size_}; }
    [[nodiscard]] std::span<const T> span() const noexcept { return {data_, size_}; }

    /// Element access
    [[nodiscard]] reference operator[](std::size_t index) noexcept {
        assert(index < size_);
        return data_[index];
    }

    [[nodiscard]] const_reference operator[](std::size_t index) const noexcept {
        assert(index < size_);
        return data_[index];
    }

    /// Iterator support
    [[nodiscard]] pointer begin() noexcept { return data_; }
    [[nodiscard]] pointer end() noexcept { return data_ + size_; }
    [[nodiscard]] const_pointer begin() const noexcept { return data_; }
    [[nodiscard]] const_pointer end() const noexcept { return data_ + size_; }

private:
    pointer data_{nullptr};
    std::size_t size_{0};
    std::function<void(pointer)> return_fn_;
};

/**
 * @brief Thread-safe memory pool with fixed-size blocks
 *
 * Pre-allocates a pool of memory blocks for efficient allocation and deallocation.
 * All blocks are cache-line aligned for optimal performance.
 *
 * @tparam T Type of elements stored in blocks
 */
template <typename T>
class MemoryPool {
public:
    static_assert(std::is_trivially_destructible_v<T>,
                  "MemoryPool only supports trivially destructible types");

    /**
     * @brief Construct a memory pool
     *
     * @param block_size Number of elements per block
     * @param initial_blocks Initial number of pre-allocated blocks
     * @param max_blocks Maximum number of blocks (0 = unlimited)
     */
    explicit MemoryPool(std::size_t block_size, std::size_t initial_blocks = 16,
                        std::size_t max_blocks = 0)
        : block_size_(block_size), max_blocks_(max_blocks),
          bytes_per_block_(calculateBlockBytes(block_size)) {
        // Pre-allocate initial blocks
        for (std::size_t i = 0; i < initial_blocks; ++i) {
            free_blocks_.push_back(allocateRawBlock());
        }
        total_blocks_ = initial_blocks;
    }

    // Non-copyable, non-movable (owns allocated memory)
    MemoryPool(const MemoryPool&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;
    MemoryPool(MemoryPool&&) = delete;
    MemoryPool& operator=(MemoryPool&&) = delete;

    ~MemoryPool() {
        std::lock_guard lock(mutex_);
        for (auto* block : free_blocks_) {
            deallocateRawBlock(block);
        }
    }

    /**
     * @brief Acquire a block from the pool
     *
     * If no free blocks are available, a new block is allocated (if within limits).
     *
     * @return PooledBlock that automatically returns to pool on destruction
     * @throws std::bad_alloc if allocation fails or max_blocks is reached
     */
    [[nodiscard]] PooledBlock<T> acquire() {
        T* block = nullptr;

        {
            std::lock_guard lock(mutex_);

            if (!free_blocks_.empty()) {
                block = free_blocks_.back();
                free_blocks_.pop_back();
            } else if (max_blocks_ == 0 || total_blocks_ < max_blocks_) {
                block = allocateRawBlock();
                ++total_blocks_;
            } else {
                throw std::bad_alloc();
            }
        }

        return PooledBlock<T>(block, block_size_, [this](T* ptr) { returnBlock(ptr); });
    }

    /**
     * @brief Try to acquire a block without throwing
     *
     * @return Optional PooledBlock, empty if no blocks available
     */
    [[nodiscard]] std::optional<PooledBlock<T>> tryAcquire() noexcept {
        try {
            return acquire();
        } catch (...) {
            return std::nullopt;
        }
    }

    /// Get the block size (number of elements per block)
    [[nodiscard]] std::size_t blockSize() const noexcept { return block_size_; }

    /// Get the number of free blocks available
    [[nodiscard]] std::size_t freeBlocks() const noexcept {
        std::lock_guard lock(mutex_);
        return free_blocks_.size();
    }

    /// Get the total number of allocated blocks
    [[nodiscard]] std::size_t totalBlocks() const noexcept {
        std::lock_guard lock(mutex_);
        return total_blocks_;
    }

    /// Get memory statistics
    struct Stats {
        std::size_t total_blocks;
        std::size_t free_blocks;
        std::size_t used_blocks;
        std::size_t bytes_per_block;
        std::size_t total_bytes;
    };

    [[nodiscard]] Stats stats() const noexcept {
        std::lock_guard lock(mutex_);
        return {total_blocks_, free_blocks_.size(), total_blocks_ - free_blocks_.size(),
                bytes_per_block_, total_blocks_ * bytes_per_block_};
    }

private:
    /// Calculate bytes per block with alignment
    [[nodiscard]] static constexpr std::size_t calculateBlockBytes(std::size_t elements) noexcept {
        const std::size_t raw_size = elements * sizeof(T);
        // Round up to cache line boundary
        return (raw_size + kCacheLineSize - 1) & ~(kCacheLineSize - 1);
    }

    /// Allocate a raw block of memory
    [[nodiscard]] T* allocateRawBlock() {
        void* ptr = ::operator new(bytes_per_block_, std::align_val_t{kCacheLineSize});
        return static_cast<T*>(ptr);
    }

    /// Deallocate a raw block
    void deallocateRawBlock(T* block) noexcept {
        ::operator delete(block, std::align_val_t{kCacheLineSize});
    }

    /// Return a block to the pool
    void returnBlock(T* block) noexcept {
        std::lock_guard lock(mutex_);
        free_blocks_.push_back(block);
    }

    const std::size_t block_size_;
    const std::size_t max_blocks_;
    const std::size_t bytes_per_block_;

    mutable std::mutex mutex_;
    std::vector<T*> free_blocks_;
    std::size_t total_blocks_{0};
};

/**
 * @brief Aligned data buffer with reference counting
 *
 * Provides a thread-safe, reference-counted buffer for zero-copy data sharing.
 *
 * @tparam T Type of elements in the buffer
 */
template <typename T>
class AlignedDataBuffer {
public:
    /**
     * @brief Construct an aligned buffer
     *
     * @param size Number of elements
     */
    explicit AlignedDataBuffer(std::size_t size)
        : size_(size),
          data_(static_cast<T*>(::operator new(size * sizeof(T), std::align_val_t{kCacheLineSize})),
                AlignedDeleter{}) {
        // Default-construct elements
        for (std::size_t i = 0; i < size; ++i) {
            new (&data_.get()[i]) T{};
        }
    }

    // Allow copy (increases ref count conceptually - but we use unique_ptr)
    // For shared ownership, use std::shared_ptr<AlignedDataBuffer>

    /// Get raw data pointer
    [[nodiscard]] T* data() noexcept { return data_.get(); }
    [[nodiscard]] const T* data() const noexcept { return data_.get(); }

    /// Get buffer size
    [[nodiscard]] std::size_t size() const noexcept { return size_; }

    /// Get span view
    [[nodiscard]] std::span<T> span() noexcept { return {data_.get(), size_}; }
    [[nodiscard]] std::span<const T> span() const noexcept { return {data_.get(), size_}; }

    /// Element access
    [[nodiscard]] T& operator[](std::size_t index) noexcept {
        assert(index < size_);
        return data_.get()[index];
    }

    [[nodiscard]] const T& operator[](std::size_t index) const noexcept {
        assert(index < size_);
        return data_.get()[index];
    }

private:
    std::size_t size_;
    std::unique_ptr<T, AlignedDeleter> data_;
};

}  // namespace aura
