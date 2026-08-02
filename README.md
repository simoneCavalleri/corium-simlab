# Corium SimLab (`corium-simlab`)

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Runtime: Corium](https://img.shields.io/badge/Runtime-Corium%20MPSC-green.svg)](https://github.com/simoneCavalleri/corium)
[![Python: Gymnasium](https://img.shields.io/badge/Python-Gymnasium-brightgreen.svg)](python/)

**Corium SimLab** is a high-performance 3D agent simulation environment and WebGPU real-time rendering engine built on top of the [Corium](https://github.com/simoneCavalleri/corium) zero-heap MPSC event-driven architecture.

Designed for Reinforcement Learning (RL), digital twins, robotic manipulators, autonomous agents, and high-frequency spatial vision sensors.

---

## Architecture & Features

- **PBR Cook-Torrance WGSL Shading**: Physically Based Rendering (GGX microfacet BRDF) with metallic, roughness, albedo, and emissive material parameters.
- **Offscreen WebGPU Vision Sensors**: 256-byte aligned staging buffer CPU readback for extracting raw RGBA visual frames for PyTorch / NumPy RL policies.
- **Forward Kinematics & Articulated Bodies**: Revolute, Prismatic, and Fixed joint support for multi-link robotic arms and manipulators.
- **Fluent C++ SceneBuilder API**: Zero-overhead programmatic 3D scene construction (`.addGroundGrid()`, `.addCube()`, `.addSphere()`, `.addModel()`, `.addJoint()`).
- **Python Gymnasium Interface (`pybind11`)**: Native C++ extension module (`corium_sim_py`) providing standard Gymnasium `CoriumEnv` (`reset()`, `step(action)`).

---

## Project Organization

```text
corium-simlab/
├── assets/                          # 3D Mesh Models and Textures
├── include/
│   └── corium_sim/                  # Public C++ Engine Headers
│       ├── kinematics/              # JointKinematics & SimJoint definitions
│       ├── physics/                 # 3D Rigid Body Physics Solver
│       ├── renderer/                # WebGPU Renderer, PBR Materials, SensorCamera
│       ├── scene/                   # SimScene & Fluent SceneBuilder API
│       ├── App.hpp                  # SimLabApp core application class
│       └── SimLab.hpp               # Single umbrella header inclusion
├── python/
│   └── corium_sim/                  # Python Gymnasium Environment Package
├── samples/                         # Standalone Executable & Script Samples
│   ├── cpp/
│   │   ├── 01_basic_environment.cpp
│   │   └── 02_robotic_arm_manipulator.cpp
│   └── python/
│       ├── 01_gym_environment.py
│       └── 02_robot_joint_control.py
├── src/                             # C++ Core Implementations (.cpp)
│   ├── kinematics/
│   ├── physics/
│   ├── python/                      # pybind11 C++ bindings
│   ├── renderer/
│   └── scene/
└── CMakeLists.txt
```

---

## Quickstart & Build Instructions

### C++ Samples & Library Build

```bash
# Configure project with C++ samples and Python bindings enabled
cmake -B build -DCORIUM_LOCAL_PATH=/home/simone/dev/corium -DBUILD_SAMPLES=ON -DBUILD_PYTHON_BINDINGS=ON

# Compile library and samples
cmake --build build

# Run C++ Realistic Industrial Robotic Manipulator Sample
./build/samples/cpp/sample_02_robotic_arm_manipulator
```

### Python Gymnasium Environment Sample

```bash
# Run Python Gymnasium Environment loop demo
python3 samples/python/01_gym_environment.py

# Run Python Robotic Joint Control & Offscreen Sensor sample
python3 samples/python/02_robot_joint_control.py
```