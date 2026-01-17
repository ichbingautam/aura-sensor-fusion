/**
 * @file bench_thread_pool.cpp
 * @brief Benchmarks for thread pool
 */

#include <aura/concurrency/thread_pool.hpp>

#include <benchmark/benchmark.h>

namespace {

using aura::TaskPriority;
using aura::ThreadPool;

static void BM_SubmitTask(benchmark::State& state) {
    aura::Logger::instance().setLevel(aura::LogLevel::Off);
    ThreadPool pool(4);

    for (auto _ : state) {
        auto future = pool.submit([]() { return 42; });
        benchmark::DoNotOptimize(future.get());
    }
}
BENCHMARK(BM_SubmitTask);

static void BM_BatchSubmit(benchmark::State& state) {
    aura::Logger::instance().setLevel(aura::LogLevel::Off);
    ThreadPool pool(static_cast<size_t>(state.range(0)));
    const int batch_size = 100;

    for (auto _ : state) {
        std::vector<std::future<int>> futures;
        futures.reserve(batch_size);
        for (int i = 0; i < batch_size; ++i) {
            futures.push_back(pool.submit([i]() { return i; }));
        }
        for (auto& f : futures) {
            benchmark::DoNotOptimize(f.get());
        }
    }

    state.SetItemsProcessed(state.iterations() * batch_size);
}
BENCHMARK(BM_BatchSubmit)->Arg(1)->Arg(2)->Arg(4)->Arg(8);

}  // namespace

BENCHMARK_MAIN();
