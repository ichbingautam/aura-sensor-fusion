# AURA: Autonomous Unified Real-time Architecture

<div align="center">

[![Build Status](https://github.com/ichbingautam/aura-sensor-fusion/workflows/CI/badge.svg)](https://github.com/ichbingautam/aura-sensor-fusion/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/std/the-standard)
[![Documentation](https://img.shields.io/badge/docs-doxygen-blue.svg)](./docs/)

**A high-performance, real-time multi-sensor fusion system for autonomous vehicles and robotics.**

[Features](#features) • [Quick Start](#quick-start) • [Architecture](#architecture) • [Documentation](#documentation) • [Contributing](#contributing)

</div>

---

## 🚀 Overview

AURA (Autonomous Unified Real-time Architecture) is a modular, template-based pipeline system designed to:

- **Normalize** heterogeneous sensor data into a unified representation
- **Synchronize** data using temporal alignment buffers
- **Fuse** information using probabilistic models
- **Distribute** processing across CPU cores
- **Visualize** real-time debugging interfaces

Built with modern C++20, AURA leverages lock-free data structures, template metaprogramming, and high-precision timing utilities for maximum performance.

## ✨ Features

### Core Capabilities

| Feature | Description | Status |
|---------|-------------|--------|
| 🔄 **Real-time Pipeline** | Modular fusion nodes with configurable input/output buffers | ✅ Implemented |
| 🔒 **Lock-free Ring Buffers** | SPSC and MPMC lock-free ring buffers with cache-line padding | ✅ Implemented |
| 🧵 **Work-stealing Thread Pool** | Priority-based task scheduling with work-stealing | ✅ Implemented |
| ⏱️ **High-precision Timestamps** | Nanosecond-precision timing with C++20 `<=>` support | ✅ Implemented |
| 🎯 **Typed Fusion Nodes** | Type-safe sensor data processing with template metaprogramming | ✅ Implemented |
| 📊 **Memory Pool Allocator** | Custom allocators for efficient memory management | ✅ Implemented |
| 📝 **Structured Logging** | Thread-safe logging with multiple severity levels | ✅ Implemented |
| 🔧 **Performance Profiling** | Scoped timers and profiling utilities | ✅ Implemented |

### Sensor Support

| Sensor Type | Data Structure | Status |
|-------------|----------------|--------|
| **LiDAR** | `LidarPoint`, `LidarFrame` | ✅ Implemented |
| **Camera** | `CameraImage` | ✅ Implemented |
| **Radar** | `RadarDetection`, `RadarFrame` | ✅ Implemented |
| **GPS/IMU** | `GpsReading`, `ImuReading` | ✅ Implemented |

### Technical Highlights

- **Template Metaprogramming**: Type-safe sensor data handling with compile-time optimization
- **Concepts & Constraints**: C++20 concepts for algorithm requirements (`FusableData`, `SensorAdapter`)
- **Modern C++20 Features**: Three-way comparison (`<=>`), `std::jthread` support, structured bindings
- **Cross-platform Threading**: Automatic fallback to `std::thread` when `std::jthread` is unavailable
- **RAII Resource Management**: Deterministic cleanup with smart pointers and scoped guards
- **Cache-aware Design**: Cache-line padding to prevent false sharing in concurrent structures

## 📋 Requirements

### Compiler Support

| Compiler | Minimum Version | Notes |
|----------|-----------------|-------|
| GCC | 10+ | Full C++20 support |
| Clang | 12+ | Full C++20 support |
| AppleClang | 14+ | Uses `std::thread` fallback |
| MSVC | 2019 (16.10+) | Full C++20 support |

### Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| CMake | 3.20+ | Build system |
| Google Test | 1.11+ | Unit testing |
| Google Benchmark | 1.6+ | Performance benchmarks |

**Optional Dependencies** (for extended features):

- Eigen3 (linear algebra)
- OpenCV 4.x (computer vision)
- PCL 1.12+ (point cloud processing)
- Boost 1.75+ (system, thread)

## 🏁 Quick Start

### Build from Source

```bash
# Clone the repository
git clone https://github.com/ichbingautam/aura-sensor-fusion.git
cd aura-sensor-fusion

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake -DCMAKE_BUILD_TYPE=Release \
      -DAURA_BUILD_TESTS=ON \
      -DAURA_BUILD_BENCHMARKS=ON \
      -DAURA_BUILD_EXAMPLES=ON \
      ..

# Build
cmake --build . --parallel $(nproc)

# Run tests
ctest --output-on-failure

# Install (optional)
sudo cmake --install .
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `AURA_BUILD_TESTS` | `ON` | Build unit tests |
| `AURA_BUILD_BENCHMARKS` | `ON` | Build performance benchmarks |
| `AURA_BUILD_EXAMPLES` | `ON` | Build example applications |
| `AURA_ENABLE_SANITIZERS` | `OFF` | Enable AddressSanitizer/UBSan |

### Basic Usage

```cpp
#include <aura/aura.hpp>

int main() {
    using namespace aura;

    // Configure logging
    Logger::instance().setLevel(LogLevel::Info);

    // Create thread pool with work-stealing
    ThreadPool pool(4);
    AURA_LOG_INFO("Thread pool started with {} threads", pool.numThreads());

    // Generate synthetic LiDAR data
    std::vector<LidarPoint> points = generatePointCloud(100000);

    // Submit parallel processing tasks
    auto future = pool.submit(TaskPriority::High, [&points]() {
        return filterGroundPoints(points, -1.5f);
    });

    // Get results
    auto filtered = future.get();
    AURA_LOG_INFO("Filtered {} -> {} points", points.size(), filtered.size());

    return 0;
}
```

### Using the Lock-free Ring Buffer

```cpp
#include <aura/concurrency/lock_free_ring_buffer.hpp>

// Single-producer single-consumer buffer (SPSC)
aura::LockFreeRingBuffer<SensorReading, 1024> spsc_buffer;

// Producer thread
spsc_buffer.tryPush(SensorReading{...});

// Consumer thread
if (auto reading = spsc_buffer.tryPop()) {
    process(*reading);
}

// Multi-producer multi-consumer buffer (MPMC)
aura::MPMCRingBuffer<SensorReading> mpmc_buffer(1024);
mpmc_buffer.tryPush(SensorReading{...});
```

### Using Fusion Nodes

```cpp
#include <aura/fusion/fusion_node.hpp>

// Create a typed fusion node
class PointCloudFilter : public aura::TypedFusionNode<LidarFrame, LidarFrame> {
public:
    PointCloudFilter() : TypedFusionNode({.name = "GroundFilter"}) {}

protected:
    std::shared_ptr<LidarFrame> processTyped(std::shared_ptr<LidarFrame> input) override {
        // Filter ground points
        auto output = std::make_shared<LidarFrame>();
        for (const auto& point : input->points) {
            if (point.z > ground_threshold_) {
                output->points.push_back(point);
            }
        }
        return output;
    }

private:
    float ground_threshold_ = -1.5f;
};
```

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         AURA Pipeline                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │
│  │  LiDAR   │  │  Camera  │  │  Radar   │  │ GPS/IMU  │           │
│  │ Adapter  │  │ Adapter  │  │ Adapter  │  │ Adapter  │           │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │
│       │             │             │             │                   │
│       ▼             ▼             ▼             ▼                   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │         Lock-Free Ring Buffers (SPSC/MPMC)                  │   │
│  │    • Cache-line padding for false sharing prevention        │   │
│  │    • Atomic sequence-based synchronization                  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │       Work-Stealing Thread Pool (TaskQueue per thread)      │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │   │
│  │  │ Fusion  │  │ Fusion  │  │ Fusion  │  │ Fusion  │        │   │
│  │  │ Node 1  │  │ Node 2  │  │ Node 3  │  │ Node N  │        │   │
│  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘        │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    Fused Output                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Module Structure

```
aura-sensor-fusion/
├── include/aura/
│   ├── aura.hpp                    # Main include header
│   ├── core/
│   │   ├── types.hpp               # Core data types (LidarPoint, etc.)
│   │   ├── timestamp.hpp           # High-precision timestamps
│   │   ├── memory_pool.hpp         # Custom memory allocators
│   │   └── logging.hpp             # Structured logging
│   ├── sensors/
│   │   ├── sensor_base.hpp         # Sensor adapter base class
│   │   ├── sensor_data.hpp         # Sensor data structures
│   │   └── sensor_traits.hpp       # Type traits for sensors
│   ├── fusion/
│   │   ├── fusion_node.hpp         # Fusion pipeline nodes
│   │   └── fusion_concepts.hpp     # C++20 concepts
│   ├── concurrency/
│   │   ├── lock_free_ring_buffer.hpp  # SPSC/MPMC ring buffers
│   │   └── thread_pool.hpp         # Work-stealing thread pool
│   └── utils/
│       └── profiler.hpp            # Performance profiling
├── src/                            # Implementation files
├── tests/
│   ├── unit/                       # Unit tests
│   │   ├── test_ring_buffer.cpp
│   │   ├── test_thread_pool.cpp
│   │   ├── test_timestamp.cpp
│   │   ├── test_memory_pool.cpp
│   │   ├── test_types.cpp
│   │   └── test_sensors.cpp
│   └── integration/                # Integration tests
├── examples/
│   ├── basic_pipeline.cpp          # Basic usage example
│   └── sensor_simulation.cpp       # Sensor simulation example
├── benchmarks/
│   ├── bench_ring_buffer.cpp
│   ├── bench_thread_pool.cpp
│   └── bench_memory_pool.cpp
└── docs/                           # Documentation
```

## 📊 Performance Targets

| Metric | Target | Status |
|--------|--------|--------|
| End-to-end Latency | < 100ms | Benchmarking in progress |
| Throughput | > 1M points/sec | Benchmarking in progress |
| Memory Usage | < 2GB peak | Benchmarking in progress |
| CPU Utilization | > 80% (8 cores) | Benchmarking in progress |
| Reliability | 99.99% uptime | Design target |

## 🧪 Testing

```bash
# Run all tests
ctest --test-dir build --output-on-failure

# Run specific test suite
./build/tests/unit/test_ring_buffer
./build/tests/unit/test_thread_pool
./build/tests/unit/test_timestamp
./build/tests/unit/test_memory_pool

# Run benchmarks
./build/benchmarks/bench_ring_buffer
./build/benchmarks/bench_thread_pool
./build/benchmarks/bench_memory_pool
```

### CI/CD Pipeline

The project includes a comprehensive CI pipeline that runs on every push:

- **Multi-platform builds**: Linux (GCC 12, Clang 15) and macOS
- **Multiple build types**: Debug and Release
- **Sanitizer testing**: AddressSanitizer and UndefinedBehaviorSanitizer
- **Code formatting**: Enforced via clang-format-17

## 📚 Documentation

- [Architecture Overview](docs/architecture.md)
- [Development Phases](docs/phases.md)
- [API Reference](docs/api/index.html)
- [Tutorials](docs/tutorials/)

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Ensure code passes formatting (`clang-format-17`)
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Inspired by real-world autonomous vehicle sensor fusion systems
- Built upon modern C++ best practices and idioms
- Thanks to the C++ community for excellent libraries and tools

---

<div align="center">

**Built with ❤️ for the autonomous vehicle community**

</div>
