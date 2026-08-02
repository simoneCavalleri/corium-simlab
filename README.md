# Corium SimLab (`corium-simlab`)

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Runtime: Corium](https://img.shields.io/badge/Runtime-Corium%20MPSC-green.svg)](https://github.com/simoneCavalleri/corium)

**Corium SimLab** is a high-performance standalone simulation application and WebGPU real-time rendering engine built on top of the [Corium](https://github.com/simoneCavalleri/corium) MPSC event-driven architecture.

Designed for digital twins, autonomous agent simulation, robotics telemetry, and high-frequency real-time spatial visualization.

---

## Architecture & Design

- **Modular C++20 Architecture**: Clean separation between interface headers (`include/corium_sim/`) and compiled C++ implementations (`src/`).
- **Zero Dynamic Heap Allocations**: Main event loop and event queues execute with zero heap allocation overhead.
- **Lock-Free MPSC Telemetry Stream**: High-frequency background thread producers (ROS2, TCP/UDP sockets, physics solvers) stream data into the main render queue without mutex contention.
- **Native WebGPU Graphics Engine**: Native WebGPU rendering backend (`wgpu-native`) with GLFW windowing and Wayland / X11 native surface creation.

---

## Directory Layout

```text
corium-simlab/
├── include/
│   └── corium_sim/
│       ├── App.hpp                  # SimLabApp product application class
│       ├── events/
│       │   └── SimEvents.hpp        # Telemetry, Camera, SimStep events
│       ├── renderer/
│       │   └── WebGpuBackend.hpp    # WebGPU backend declaration
│       └── services/
│           └── TelemetryService.hpp # Background telemetry streaming service
├── src/
│   ├── main.cpp                     # Standalone main executable entry point
│   ├── App.cpp                      # SimLabApp implementation
│   ├── renderer/
│   │   └── WebGpuBackend.cpp        # WebGPU pipeline implementation
│   └── services/
│       └── TelemetryService.cpp    # Telemetry service implementation
└── CMakeLists.txt
```

---

## Build & Run

```bash
# Configure build using CMake
cmake -B build

# Build standalone executable (corium-simlab)
cmake --build build

# Run application
LD_LIBRARY_PATH=./build:_deps/wgpu_native-src ./build/corium-simlab
```

### Local Corium Development

```bash
cmake -B build -DCORIUM_LOCAL_PATH=../corium
cmake --build build
LD_LIBRARY_PATH=./build:_deps/wgpu_native-src ./build/corium-simlab
```