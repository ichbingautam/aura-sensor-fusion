cmake_minimum_required(VERSION 3.20)

# Find or fetch Google Test
include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()
include(GoogleTest)

# ============================================================================
# Unit Tests
# ============================================================================

# Ring buffer tests
add_executable(test_ring_buffer unit/test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_ring_buffer)

# Thread pool tests
add_executable(test_thread_pool unit/test_thread_pool.cpp)
target_link_libraries(test_thread_pool
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_thread_pool)

# Pipeline coordinator tests
add_executable(test_pipeline_coordinator unit/test_pipeline_coordinator.cpp)
target_link_libraries(test_pipeline_coordinator
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_pipeline_coordinator)

# Kalman Filter tests
add_executable(test_kalman_filter unit/test_kalman_filter.cpp)
target_link_libraries(test_kalman_filter
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_kalman_filter)

# Extended Kalman Filter tests
add_executable(test_ekf unit/test_ekf.cpp)
target_link_libraries(test_ekf
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_ekf)

# Unscented Kalman Filter tests
add_executable(test_ukf unit/test_ukf.cpp)
target_link_libraries(test_ukf
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_ukf)

# Particle Filter tests
add_executable(test_particle_filter unit/test_particle_filter.cpp)
target_link_libraries(test_particle_filter
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_particle_filter)

# Data Association tests
add_executable(test_data_association unit/test_data_association.cpp)
target_link_libraries(test_data_association
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_data_association)

# Memory pool tests
add_executable(test_memory_pool unit/test_memory_pool.cpp)
target_link_libraries(test_memory_pool
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_memory_pool)

# Types tests
add_executable(test_types unit/test_types.cpp)
target_link_libraries(test_types
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_types)

# Timestamp tests
add_executable(test_timestamp unit/test_timestamp.cpp)
target_link_libraries(test_timestamp
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_timestamp)

# Sensor tests
add_executable(test_sensors unit/test_sensors.cpp)
target_link_libraries(test_sensors
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_sensors)

# ============================================================================
# Integration Tests
# ============================================================================

add_executable(test_pipeline_integration integration/test_pipeline.cpp)
target_link_libraries(test_pipeline_integration
    PRIVATE
        aura_core
        GTest::gtest_main
)
gtest_discover_tests(test_pipeline_integration)
