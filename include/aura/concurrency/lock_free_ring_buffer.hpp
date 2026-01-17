/**
 * @file lock_free_ring_buffer.hpp
 * @brief Lock-free single-producer single-consumer ring buffer
 *
 * Provides a high-performance, lock-free ring buffer for concurrent
 * data transfer between threads. Uses atomic operations and memory
 * ordering to ensure thread safety without locks.
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/core/memory_pool.hpp>

#include <atomic>
#include <cassert>
#include <cstddef>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>

namespace aura {

/**
 * @brief Lock-free single-producer single-consumer (SPSC) ring buffer
 *
 * This implementation uses cache-line padding to prevent false sharing
 * between producer and consumer indices.
 *
 * @tparam T Type of elements stored in the buffer
 * @tparam Capacity Maximum number of elements (must be power of 2)
 *
 * @note This buffer is designed for single-producer single-consumer scenarios.
 *       For multiple producers or consumers, external synchronization is required.
 */
template <typename T, std::size_t Capacity>
    requires(Capacity > 0 && (Capacity & (Capacity - 1)) == 0)  // Power of 2
class LockFreeRingBuffer {
public:
    static_assert(std::is_nothrow_move_constructible_v<T>, "T must be nothrow move constructible");
    static_assert(std::is_nothrow_move_assignable_v<T>, "T must be nothrow move assignable");

    using value_type = T;
    using size_type = std::size_t;

    /// Mask for fast modulo operation (works because Capacity is power of 2)
    static constexpr size_type kMask = Capacity - 1;

    LockFreeRingBuffer() : buffer_(Capacity) {
        // Initialize atomic indices
        write_idx_.store(0, std::memory_order_relaxed);
        read_idx_.store(0, std::memory_order_relaxed);
    }

    // Non-copyable, non-movable
    LockFreeRingBuffer(const LockFreeRingBuffer&) = delete;
    LockFreeRingBuffer& operator=(const LockFreeRingBuffer&) = delete;
    LockFreeRingBuffer(LockFreeRingBuffer&&) = delete;
    LockFreeRingBuffer& operator=(LockFreeRingBuffer&&) = delete;

    ~LockFreeRingBuffer() = default;

    /**
     * @brief Push an element to the buffer
     *
     * @param item Element to push (moved)
     * @return true if successful, false if buffer is full
     */
    [[nodiscard]] bool tryPush(T&& item) noexcept {
        const size_type write_idx = write_idx_.load(std::memory_order_relaxed);
        const size_type next_write_idx = (write_idx + 1) & kMask;

        // Check if buffer is full
        if (next_write_idx == read_idx_.load(std::memory_order_acquire)) {
            return false;
        }

        // Store the item
        buffer_[write_idx] = std::move(item);

        // Publish the write
        write_idx_.store(next_write_idx, std::memory_order_release);
        return true;
    }

    /**
     * @brief Push an element (copy variant)
     */
    [[nodiscard]] bool tryPush(const T& item) noexcept(std::is_nothrow_copy_constructible_v<T>) {
        T copy = item;
        return tryPush(std::move(copy));
    }

    /**
     * @brief Construct element in-place
     *
     * @tparam Args Constructor argument types
     * @param args Arguments to forward to T's constructor
     * @return true if successful, false if buffer is full
     */
    template <typename... Args>
    [[nodiscard]] bool tryEmplace(Args&&... args) noexcept(
        std::is_nothrow_constructible_v<T, Args...>) {
        const size_type write_idx = write_idx_.load(std::memory_order_relaxed);
        const size_type next_write_idx = (write_idx + 1) & kMask;

        if (next_write_idx == read_idx_.load(std::memory_order_acquire)) {
            return false;
        }

        buffer_[write_idx] = T(std::forward<Args>(args)...);
        write_idx_.store(next_write_idx, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop an element from the buffer
     *
     * @return The popped element, or nullopt if buffer is empty
     */
    [[nodiscard]] std::optional<T> tryPop() noexcept {
        const size_type read_idx = read_idx_.load(std::memory_order_relaxed);

        // Check if buffer is empty
        if (read_idx == write_idx_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }

        // Move out the item
        T item = std::move(buffer_[read_idx]);

        // Publish the read
        read_idx_.store((read_idx + 1) & kMask, std::memory_order_release);
        return item;
    }

    /**
     * @brief Pop an element into a reference
     *
     * @param[out] item Reference to store the popped element
     * @return true if successful, false if buffer is empty
     */
    [[nodiscard]] bool tryPop(T& item) noexcept {
        auto result = tryPop();
        if (result) {
            item = std::move(*result);
            return true;
        }
        return false;
    }

    /**
     * @brief Peek at the front element without removing it
     *
     * @return Pointer to front element, or nullptr if empty
     * @warning The returned pointer is only valid until the next pop operation
     */
    [[nodiscard]] const T* peek() const noexcept {
        const size_type read_idx = read_idx_.load(std::memory_order_relaxed);
        if (read_idx == write_idx_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return &buffer_[read_idx];
    }

    /**
     * @brief Check if the buffer is empty
     *
     * @note This is a snapshot and may be stale by the time it's used
     */
    [[nodiscard]] bool empty() const noexcept {
        return read_idx_.load(std::memory_order_acquire) ==
               write_idx_.load(std::memory_order_acquire);
    }

    /**
     * @brief Check if the buffer is full
     *
     * @note This is a snapshot and may be stale by the time it's used
     */
    [[nodiscard]] bool full() const noexcept {
        const size_type write_idx = write_idx_.load(std::memory_order_acquire);
        const size_type read_idx = read_idx_.load(std::memory_order_acquire);
        return ((write_idx + 1) & kMask) == read_idx;
    }

    /**
     * @brief Get the current number of elements
     *
     * @note This is a snapshot and may be stale by the time it's used
     */
    [[nodiscard]] size_type size() const noexcept {
        const size_type write_idx = write_idx_.load(std::memory_order_acquire);
        const size_type read_idx = read_idx_.load(std::memory_order_acquire);
        return (write_idx - read_idx + Capacity) & kMask;
    }

    /**
     * @brief Get the maximum capacity
     */
    [[nodiscard]] static constexpr size_type capacity() noexcept { return Capacity - 1; }

    /**
     * @brief Clear all elements from the buffer
     *
     * @warning Not thread-safe! Only call when no other threads are accessing the buffer.
     */
    void clear() noexcept {
        read_idx_.store(write_idx_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }

private:
    std::vector<T> buffer_;

    // Cache-line aligned indices to prevent false sharing
    alignas(kCacheLineSize) std::atomic<size_type> write_idx_{0};
    alignas(kCacheLineSize) std::atomic<size_type> read_idx_{0};
};

/**
 * @brief Lock-free multi-producer multi-consumer (MPMC) ring buffer
 *
 * Uses a sequence-based approach for thread safety with multiple
 * producers and consumers.
 *
 * @tparam T Type of elements stored in the buffer
 */
template <typename T>
class MPMCRingBuffer {
public:
    static_assert(std::is_nothrow_move_constructible_v<T>, "T must be nothrow move constructible");

    /**
     * @brief Construct buffer with given capacity
     *
     * @param capacity Buffer capacity (will be rounded up to power of 2)
     */
    explicit MPMCRingBuffer(std::size_t capacity)
        : capacity_(nextPowerOfTwo(capacity)), mask_(capacity_ - 1),
          slots_(std::make_unique<Slot[]>(capacity_)) {
        for (std::size_t i = 0; i < capacity_; ++i) {
            slots_[i].sequence.store(i, std::memory_order_relaxed);
        }
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    // Non-copyable, non-movable
    MPMCRingBuffer(const MPMCRingBuffer&) = delete;
    MPMCRingBuffer& operator=(const MPMCRingBuffer&) = delete;
    MPMCRingBuffer(MPMCRingBuffer&&) = delete;
    MPMCRingBuffer& operator=(MPMCRingBuffer&&) = delete;

    /**
     * @brief Try to push an element
     *
     * @param item Element to push
     * @return true if successful, false if full
     */
    [[nodiscard]] bool tryPush(T&& item) noexcept {
        Slot* slot;
        std::size_t pos = head_.load(std::memory_order_relaxed);

        for (;;) {
            slot = &slots_[pos & mask_];
            std::size_t seq = slot->sequence.load(std::memory_order_acquire);
            auto diff = static_cast<std::ptrdiff_t>(seq) - static_cast<std::ptrdiff_t>(pos);

            if (diff == 0) {
                // Slot is ready, try to claim it
                if (head_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    break;
                }
            } else if (diff < 0) {
                // Buffer is full
                return false;
            } else {
                // Another producer got here first, retry
                pos = head_.load(std::memory_order_relaxed);
            }
        }

        slot->value = std::move(item);
        slot->sequence.store(pos + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Try to pop an element
     *
     * @return The popped element, or nullopt if empty
     */
    [[nodiscard]] std::optional<T> tryPop() noexcept {
        Slot* slot;
        std::size_t pos = tail_.load(std::memory_order_relaxed);

        for (;;) {
            slot = &slots_[pos & mask_];
            std::size_t seq = slot->sequence.load(std::memory_order_acquire);
            auto diff = static_cast<std::ptrdiff_t>(seq) - static_cast<std::ptrdiff_t>(pos + 1);

            if (diff == 0) {
                // Slot has data, try to claim it
                if (tail_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    break;
                }
            } else if (diff < 0) {
                // Buffer is empty
                return std::nullopt;
            } else {
                // Another consumer got here first, retry
                pos = tail_.load(std::memory_order_relaxed);
            }
        }

        T item = std::move(slot->value);
        slot->sequence.store(pos + mask_ + 1, std::memory_order_release);
        return item;
    }

    /// Check if empty (approximate)
    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    /// Get capacity
    [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }

private:
    static std::size_t nextPowerOfTwo(std::size_t n) noexcept {
        --n;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        n |= n >> 32;
        return ++n;
    }

    struct Slot {
        std::atomic<std::size_t> sequence;
        T value;
    };

    const std::size_t capacity_;
    const std::size_t mask_;
    std::unique_ptr<Slot[]> slots_;

    alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};
    alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};
};

}  // namespace aura
