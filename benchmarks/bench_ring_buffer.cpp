/**
 * @file bench_ring_buffer.cpp
 * @brief Benchmarks for lock-free ring buffer
 */

#include <aura/concurrency/lock_free_ring_buffer.hpp>

#include <benchmark/benchmark.h>

namespace {

using aura::LockFreeRingBuffer;
using aura::MPMCRingBuffer;

static void BM_SPSCPushPop(benchmark::State& state) {
    LockFreeRingBuffer<int, 1024> buffer;

    for (auto _ : state) {
        buffer.tryPush(42);
        benchmark::DoNotOptimize(buffer.tryPop());
    }
}
BENCHMARK(BM_SPSCPushPop);

static void BM_SPSCBatchPush(benchmark::State& state) {
    LockFreeRingBuffer<int, 1024> buffer;
    const auto batch_size = state.range(0);

    for (auto _ : state) {
        for (int i = 0; i < batch_size; ++i) {
            buffer.tryPush(i);
        }
        for (int i = 0; i < batch_size; ++i) {
            benchmark::DoNotOptimize(buffer.tryPop());
        }
    }

    state.SetItemsProcessed(state.iterations() * batch_size);
}
BENCHMARK(BM_SPSCBatchPush)->Range(8, 512);

static void BM_MPMCPushPop(benchmark::State& state) {
    MPMCRingBuffer<int> buffer(1024);

    for (auto _ : state) {
        buffer.tryPush(42);
        benchmark::DoNotOptimize(buffer.tryPop());
    }
}
BENCHMARK(BM_MPMCPushPop);

}  // namespace

BENCHMARK_MAIN();
