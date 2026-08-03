# Corium SimLab (`corium-simlab`)

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Runtime: Corium](https://img.shields.io/badge/Runtime-Corium%20MPSC-green.svg)](https://github.com/simoneCavalleri/corium)
[![Python: Gymnasium](https://img.shields.io/badge/Python-Gymnasium-brightgreen.svg)](python/)
[![Graphics: WebGPU](https://img.shields.io/badge/Graphics-WebGPU%20WGSL-orange.svg)](src/renderer/)

**Corium SimLab** is a high-performance **C++20 Physical Agent Incubator & Simulation Framework** built on top of the zero-heap [Corium](https://github.com/simoneCavalleri/corium) MPSC event-driven architecture and real-time WebGPU rendering engine.

Designed for Embodied AI, Reinforcement Learning (RL), mobile robots, manipulators, spatial vision sensors, and high-frequency digital twins.

---

## 🏛️ Physical Agent Incubator Paradigm

Corium SimLab is structured around the **Physical Agent — Environment** feedback loop:

```text
 ┌──────────────────────────────────────────────────────────┐
 │                   SimEnvironment                         │
 │  (World Physics, Terrains, Tasks, Reward, Reset)         │
 └─────────────┬──────────────────────────────▲─────────────┘
               │                              │
     Physical State & Vision                Actions (Torque, Vel, Pos)
               │                              │
 ┌─────────────▼──────────────┐    ┌──────────┴─────────────┐
 │     Perception / Sensors   │    │    Actuator System     │
 │ (LiDAR, Camera, IMU, Joint)│    │  (Motors, Grippers)    │
 └─────────────┬──────────────┘    └──────────▲─────────────┘
               │                              │
    Zero-Heap Observation Span         Action Vector Span
               │                              │
 ┌─────────────▼──────────────────────────────┴─────────────┐
 │                Brain / Planner / Policy                  │
 │    (Python PyTorch/RL, C++ IK/MPC, Trajectory Planner)   │
 └──────────────────────────────────────────────────────────┘
```

---

## 🚀 Key Features & Highlights

- **Compile-Time Physical Agent Monomorphization (`PhysicalAgent`)**: Static C++20 agent composition eliminating virtual function dispatch (`vtable`) and dynamic heap allocations in simulation inner-loops.
- **Zero-Allocation Sensor Suites (`SensorSuite`)**: Compile-time variadic containers aggregating multi-modal observations (3D LiDAR, IMU, Joint Encoders, RGBD cameras) into contiguous zero-copy memory arrays.
- **Compile-Time Actuator Suites (`ActuatorSuite`)**: Zero-heap action spaces mapping normalized action vectors to joint motors, wheel drives, and physical forces.
- **Physical Environments & Tasks (`SimEnvironment`, `ITask`)**: Modular environment wrapper holding world physics, reset dynamics, reward calculation, and step execution.
- **PBR Cook-Torrance WGSL Rendering**: Physically Based Rendering (GGX microfacet BRDF) with metallic, roughness, albedo, and emissive materials in WebGPU.
- **Offscreen WebGPU Vision Sensors**: 256-byte aligned staging buffer CPU readback for extracting raw RGBA visual frames for PyTorch / NumPy RL policies.
- **Vectorized Parallel Environments (`VectorEnv`)**: High-performance multi-instance manager executing $N$ 3D simulation environments in parallel for ultra-fast policy training.
- **Python Gymnasium Interface (`pybind11`)**: Native C++ extension module (`corium_sim_py`) exposing standard Gymnasium `CoriumEnv` and vectorized environments.

---

## 📁 Project Domain Organization

```text
corium-simlab/
├── assets/                          # 3D Mesh Models, Textures, and URDF Specifications
├── include/
│   └── corium_sim/                  # Public C++ Engine Headers
│       ├── agent/                   # Physical Agent Incubator (Compile-Time C++20)
│       │   ├── ActuatorSuite.hpp    # Compile-time variadic actuator containers
│       │   ├── Actuators.hpp        # Joint & differential drive actuators
│       │   ├── Concepts.hpp         # C++20 Sensor, Actuator, Environment concepts
│       │   ├── PhysicalAgent.hpp    # Monomorphized PhysicalAgent & AgentBuilder
│       │   ├── SensorSuite.hpp      # Compile-time variadic sensor containers
│       │   └── Sensors.hpp          # 3D LiDAR, IMU, JointEncoder sensors
│       ├── environment/             # Simulation Environments & Training Tasks
│       │   ├── SimEnvironment.hpp   # 3D Physical environment step/reset loop
│       │   └── Task.hpp             # Reward, done & goal termination contract
│       ├── kinematics/              # JointKinematics (FK & IK Solvers)
│       ├── physics/                 # 3D Physics Engine & Raycast LiDAR
│       ├── renderer/                # WebGPU Renderer, PBR Materials, SensorCamera
│       ├── scene/                   # SimScene, SceneBuilder & UrdfLoader
│       └── SimLab.hpp               # Master umbrella header inclusion
├── python/
│   └── corium_sim/                  # Python Gymnasium & VectorEnv Package
├── samples/                         # Executable & Script Samples
│   ├── cpp/
│   │   ├── 01_physical_agent_incubator.cpp   # Compile-time Physical Agent Incubator
│   │   ├── 02_robotic_arm_manipulator.cpp    # URDF Robot arm joint control & IK
│   └── python/
│       ├── 01_gym_environment.py             # Gymnasium step loop
│       ├── 02_robot_joint_control.py         # Joint control & offscreen rendering
│       ├── 03_train_rl_agent.py              # IK target reach RL training
│       ├── 04_urdf_robot_loader.py           # URDF parsing and display
│       ├── 05_lidar_sensor_scan.py           # 3D LiDAR point cloud scan
│       └── 06_vectorized_environments.py    # 8 Parallel vectorized environments
├── src/                             # C++ Core Implementations (.cpp)
└── CMakeLists.txt
```

---

## 💻 Quickstart & Build Instructions

### System Prerequisites (Linux)

```bash
# Ubuntu / Debian
sudo apt update
sudo apt install cmake build-essential pkg-config python3-dev libxkbcommon-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libwayland-dev wayland-protocols
```

### C++ Samples & Library Build

```bash
# Configure project with C++ samples enabled
cmake -B build -DCORIUM_LOCAL_PATH=/home/simone/dev/corium -DBUILD_SAMPLES=ON -DBUILD_PYTHON_BINDINGS=OFF

# Compile library and samples
cmake --build build

# Run C++ Physical Agent Incubator Sample (Compile-Time C++20)
./build/samples/cpp/sample_01_physical_agent_incubator

# Run C++ Industrial Robotic Manipulator Sample
./build/samples/cpp/sample_02_robotic_arm_manipulator
```

### Python Package Installation & Gymnasium Training

```bash
# Install corium-sim package in editable mode
pip install -e . --break-system-packages

# Run Gymnasium environment sample
python3 samples/python/01_gym_environment.py

# Run parallel vectorized 3D environments (8 environments)
python3 samples/python/06_vectorized_environments.py
```