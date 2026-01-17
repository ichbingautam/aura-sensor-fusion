/**
 * @file bench_memory_pool.cpp
 * @brief Benchmarks for memory pool
 */

#include <aura/core/memory_pool.hpp>

#include <benchmark/benchmark.h>

namespace {

using aura::MemoryPool;

static void BM_AcquireRelease(benchmark::State& state) {
    MemoryPool<float> pool(1024, 16);

    for (auto _ : state) {
        auto block = pool.acquire();
        benchmark::DoNotOptimize(block.data());
    }
}
BENCHMARK(BM_AcquireRelease);

static void BM_AcquireWriteRelease(benchmark::State& state) {
    MemoryPool<float> pool(static_cast<size_t>(state.range(0)), 16);

    for (auto _ : state) {
        auto block = pool.acquire();
        for (size_t i = 0; i < block.size(); ++i) {
            block[i] = static_cast<float>(i);
        }
        benchmark::DoNotOptimize(block.data());
    }

    state.SetBytesProcessed(state.iterations() * state.range(0) * sizeof(float));
}
BENCHMARK(BM_AcquireWriteRelease)->Range(64, 4096);

static void BM_NewDelete(benchmark::State& state) {
    const auto size = static_cast<size_t>(state.range(0));

    for (auto _ : state) {
        auto* ptr = new float[size];
        benchmark::DoNotOptimize(ptr);
        delete[] ptr;
    }
}
BENCHMARK(BM_NewDelete)->Range(64, 4096);

}  // namespace

BENCHMARK_MAIN();
