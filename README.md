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

Built with modern C++20/23, AURA leverages lock-free data structures, template metaprogramming, and SIMD-optimized processing kernels for maximum performance.

## ✨ Features

### Core Capabilities

| Feature | Description |
|---------|-------------|
| 🔄 **Real-time Fusion** | Process LiDAR, camera, radar, and GPS data with <100ms latency |
| 🔒 **Lock-free Pipelines** | Zero-copy data transfer with atomic operations |
| ⚡ **SIMD Acceleration** | AVX2/AVX-512 optimized processing kernels |
| 🧵 **Work-stealing Scheduler** | Efficient multi-threaded task distribution |
| 📊 **Temporal Alignment** | Synchronize asynchronous sensor streams |
| 🎯 **Multi-hypothesis Tracking** | Probabilistic object tracking and prediction |

### Technical Highlights

- **Template Metaprogramming**: Type-safe sensor data handling with compile-time optimization
- **Concepts & Constraints**: C++20 concepts for algorithm requirements
- **Coroutines**: Asynchronous data processing with C++20 coroutines
- **Custom Allocators**: Memory pool allocators for zero-copy data transfer
- **RAII Resource Management**: Deterministic cleanup with smart pointers

## 📋 Requirements

### Compiler Support

| Compiler | Minimum Version |
|----------|-----------------|
| GCC | 10+ |
| Clang | 12+ |
| MSVC | 2019 (16.10+) |

### Dependencies

- CMake 3.20+
- Eigen3 (linear algebra)
- OpenCV 4.x (computer vision)
- PCL 1.12+ (point cloud processing)
- Boost 1.75+ (system, thread)
- Google Test (testing)
- Google Benchmark (performance)

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
      ..

# Build
cmake --build . --parallel $(nproc)

# Run tests
ctest --output-on-failure

# Install (optional)
sudo cmake --install .
```

### Docker Build

```bash
# Build Docker image
docker build -t aura-fusion:latest .

# Run container
docker run -it --rm aura-fusion:latest
```

### Basic Usage

```cpp
#include <aura/core/pipeline.hpp>
#include <aura/sensors/lidar_adapter.hpp>
#include <aura/fusion/kalman_filter.hpp>

int main() {
    using namespace aura;

    // Create fusion pipeline
    auto pipeline = Pipeline::Builder()
        .addSensor<LidarAdapter>("velodyne_front")
        .addSensor<CameraAdapter>("camera_main")
        .addFusionNode<KalmanFilter>()
        .build();

    // Start processing
    pipeline.start();

    // Process sensor data
    while (auto result = pipeline.getResult()) {
        // Use fused data
        auto fused_objects = result->objects();
    }

    return 0;
}
```

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         AURA Pipeline                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │
│  │  LiDAR   │  │  Camera  │  │  Radar   │  │   GPS    │           │
│  │ Adapter  │  │ Adapter  │  │ Adapter  │  │ Adapter  │           │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │
│       │             │             │             │                   │
│       ▼             ▼             ▼             ▼                   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              Lock-Free Ring Buffers                         │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              Temporal Alignment Module                       │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │    Work-Stealing Thread Pool (Processing Nodes)             │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │   │
│  │  │ Point   │  │ Object  │  │ Fusion  │  │Tracking │        │   │
│  │  │ Cloud   │  │Detection│  │ Node    │  │  Node   │        │   │
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
│   ├── core/           # Core types, memory management, utilities
│   ├── sensors/        # Sensor adapters and data types
│   ├── fusion/         # Fusion algorithms (Kalman, Bayesian)
│   ├── concurrency/    # Thread pool, lock-free structures
│   └── utils/          # Logging, profiling, math utilities
├── src/                # Implementation files
├── tests/              # Unit and integration tests
├── examples/           # Usage examples
├── benchmarks/         # Performance benchmarks
└── docs/               # Documentation
```

## 📊 Performance Targets

| Metric | Target | Current |
|--------|--------|---------|
| End-to-end Latency | < 100ms | TBD |
| Throughput | > 1M points/sec | TBD |
| Memory Usage | < 2GB peak | TBD |
| CPU Utilization | > 80% (8 cores) | TBD |
| Reliability | 99.99% uptime | TBD |

## 📚 Documentation

- [Architecture Overview](docs/architecture.md)
- [Development Phases](docs/phases.md)
- [API Reference](docs/api/index.html)
- [Tutorials](docs/tutorials/)

## 🧪 Testing

```bash
# Run all tests
ctest --output-on-failure

# Run specific test suite
./build/tests/unit/test_ring_buffer
./build/tests/unit/test_thread_pool

# Run benchmarks
./build/benchmarks/bench_fusion_pipeline
```

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

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
