cmake_minimum_required(VERSION 3.20)

# Project definition
project(aura-sensor-fusion
    VERSION 1.0.0
    DESCRIPTION "AURA: Real-time Multi-Sensor Fusion System for Autonomous Vehicles"
    HOMEPAGE_URL "https://github.com/ichbingautam/aura-sensor-fusion"
    LANGUAGES CXX
)

# ============================================================================
# Build Options
# ============================================================================
option(AURA_BUILD_TESTS "Build unit and integration tests" ON)
option(AURA_BUILD_BENCHMARKS "Build performance benchmarks" ON)
option(AURA_BUILD_EXAMPLES "Build example applications" ON)
option(AURA_BUILD_DOCS "Build documentation" OFF)
option(AURA_ENABLE_SANITIZERS "Enable AddressSanitizer and UBSan" OFF)
option(AURA_ENABLE_COVERAGE "Enable code coverage" OFF)
option(AURA_USE_SIMD "Enable SIMD optimizations" ON)

# ============================================================================
# C++ Standard Configuration
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDEs and tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Compiler Warnings and Flags
# ============================================================================
add_library(aura_compiler_flags INTERFACE)

target_compile_features(aura_compiler_flags INTERFACE cxx_std_20)

# Compiler-specific warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(aura_compiler_flags INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wimplicit-fallthrough
    )

    # GCC-specific warnings
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(aura_compiler_flags INTERFACE
            -Wmisleading-indentation
            -Wduplicated-cond
            -Wduplicated-branches
            -Wlogical-op
            -Wuseless-cast
        )
    endif()

    # SIMD flags
    if(AURA_USE_SIMD)
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
        check_cxx_compiler_flag("-mavx512f" COMPILER_SUPPORTS_AVX512)

        if(COMPILER_SUPPORTS_AVX2)
            target_compile_options(aura_compiler_flags INTERFACE -mavx2)
            target_compile_definitions(aura_compiler_flags INTERFACE AURA_HAS_AVX2)
        endif()
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(aura_compiler_flags INTERFACE
        /W4
        /WX
        /permissive-
        /Zc:__cplusplus
    )

    if(AURA_USE_SIMD)
        target_compile_options(aura_compiler_flags INTERFACE /arch:AVX2)
        target_compile_definitions(aura_compiler_flags INTERFACE AURA_HAS_AVX2)
    endif()
endif()

# ============================================================================
# Sanitizers
# ============================================================================
if(AURA_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(aura_compiler_flags INTERFACE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
        )
        target_link_options(aura_compiler_flags INTERFACE
            -fsanitize=address,undefined
        )
    endif()
endif()

# ============================================================================
# Code Coverage
# ============================================================================
if(AURA_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(aura_compiler_flags INTERFACE --coverage)
        target_link_options(aura_compiler_flags INTERFACE --coverage)
    endif()
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(Threads REQUIRED)

# Optional dependencies
find_package(Eigen3 3.4 QUIET)
find_package(OpenCV 4 QUIET)
find_package(PCL 1.12 QUIET)
find_package(Boost 1.75 QUIET COMPONENTS system thread)

# ============================================================================
# AURA Core Library
# ============================================================================
add_library(aura_core STATIC
    # Core sources
    src/core/types.cpp
    src/core/timestamp.cpp
    src/core/memory_pool.cpp
    src/core/logging.cpp

    # Concurrency sources
    src/concurrency/thread_pool.cpp

    # Sensor sources
    src/sensors/sensor_base.cpp

    # Fusion sources
    src/fusion/fusion_node.cpp

    # Utils sources
    src/utils/profiler.cpp
)

target_include_directories(aura_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(aura_core
    PUBLIC
        aura_compiler_flags
        Threads::Threads
)

# Link optional dependencies if found
if(Eigen3_FOUND)
    target_link_libraries(aura_core PUBLIC Eigen3::Eigen)
    target_compile_definitions(aura_core PUBLIC AURA_HAS_EIGEN)
endif()

if(OpenCV_FOUND)
    target_link_libraries(aura_core PUBLIC ${OpenCV_LIBS})
    target_compile_definitions(aura_core PUBLIC AURA_HAS_OPENCV)
endif()

if(Boost_FOUND)
    target_link_libraries(aura_core PUBLIC Boost::system Boost::thread)
    target_compile_definitions(aura_core PUBLIC AURA_HAS_BOOST)
endif()

# Set library properties
set_target_properties(aura_core PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    POSITION_INDEPENDENT_CODE ON
)

# Create alias for consistent naming
add_library(aura::core ALIAS aura_core)

# ============================================================================
# Examples
# ============================================================================
if(AURA_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ============================================================================
# Tests
# ============================================================================
if(AURA_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ============================================================================
# Benchmarks
# ============================================================================
if(AURA_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# ============================================================================
# Documentation
# ============================================================================
if(AURA_BUILD_DOCS)
    find_package(Doxygen REQUIRED)
    add_subdirectory(docs)
endif()

# ============================================================================
# Installation
# ============================================================================
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install targets
install(TARGETS aura_core aura_compiler_flags
    EXPORT aura-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(DIRECTORY include/aura
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install CMake config files
install(EXPORT aura-targets
    FILE aura-targets.cmake
    NAMESPACE aura::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aura
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/aura-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/aura-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aura
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/aura-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/aura-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/aura-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aura
)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "AURA Sensor Fusion Configuration Summary")
message(STATUS "========================================")
message(STATUS "Version:           ${PROJECT_VERSION}")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Options:")
message(STATUS "  Build tests:     ${AURA_BUILD_TESTS}")
message(STATUS "  Build benchmarks:${AURA_BUILD_BENCHMARKS}")
message(STATUS "  Build examples:  ${AURA_BUILD_EXAMPLES}")
message(STATUS "  Build docs:      ${AURA_BUILD_DOCS}")
message(STATUS "  Enable sanitizers: ${AURA_ENABLE_SANITIZERS}")
message(STATUS "  Enable coverage: ${AURA_ENABLE_COVERAGE}")
message(STATUS "  Use SIMD:        ${AURA_USE_SIMD}")
message(STATUS "")
message(STATUS "Optional Dependencies:")
message(STATUS "  Eigen3:          ${Eigen3_FOUND}")
message(STATUS "  OpenCV:          ${OpenCV_FOUND}")
message(STATUS "  PCL:             ${PCL_FOUND}")
message(STATUS "  Boost:           ${Boost_FOUND}")
message(STATUS "")
