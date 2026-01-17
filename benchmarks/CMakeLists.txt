cmake_minimum_required(VERSION 3.20)

# Find Google Benchmark
include(FetchContent)

FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.8.3
)
set(BENCHMARK_ENABLE_TESTING OFF)
FetchContent_MakeAvailable(googlebenchmark)

# Ring buffer benchmark
add_executable(bench_ring_buffer bench_ring_buffer.cpp)
target_link_libraries(bench_ring_buffer
    PRIVATE
        aura_core
        benchmark::benchmark
)

# Thread pool benchmark
add_executable(bench_thread_pool bench_thread_pool.cpp)
target_link_libraries(bench_thread_pool
    PRIVATE
        aura_core
        benchmark::benchmark
)

# Memory pool benchmark
add_executable(bench_memory_pool bench_memory_pool.cpp)
target_link_libraries(bench_memory_pool
    PRIVATE
        aura_core
        benchmark::benchmark
)
