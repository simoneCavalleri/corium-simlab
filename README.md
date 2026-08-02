# Corium SimLab (`corium-simlab`)

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Runtime: Corium](https://img.shields.io/badge/Runtime-Corium%20MPSC-green.svg)](https://github.com/simoneCavalleri/corium)
[![Python: Gymnasium](https://img.shields.io/badge/Python-Gymnasium-brightgreen.svg)](python/)
[![Graphics: WebGPU](https://img.shields.io/badge/Graphics-WebGPU%20WGSL-orange.svg)](src/renderer/)

**Corium SimLab** is a high-performance 3D agent simulation environment and WebGPU real-time rendering engine built on top of the [Corium](https://github.com/simoneCavalleri/corium) zero-heap MPSC event-driven architecture.

Designed for Reinforcement Learning (RL), digital twins, industrial robotic manipulators, autonomous agents, and high-frequency spatial vision sensors.

---

## Key Features & Highlights

- **PBR Cook-Torrance WGSL Shading**: Physically Based Rendering (GGX microfacet BRDF) with metallic, roughness, albedo, and emissive material parameters.
- **Offscreen WebGPU Vision Sensors**: 256-byte aligned staging buffer CPU readback for extracting raw RGBA visual frames for PyTorch / NumPy RL policies.
- **Inverse Kinematics (IK Solver)**: Jacobian Transpose / Gradient Descent solver for multi-joint robotic arms (`solve_ik(end_effector, x, y, z)`).
- **URDF XML Parser & Robot Loader**: Native parser for loading standard URDF robot specifications (`<link>`, `<joint>`, `<geometry>`, `<origin>`, `<limit>`).
- **3D LiDAR & Ultrasonic Sensor Raycasting**: 3D Ray-AABB slab intersection engine and 360-degree LiDAR point cloud scanning returning distance, hit points, and surface normals.
- **Vectorized Parallel Environments (`VectorEnv`)**: High-performance multi-instance manager executing $N$ 3D simulation environments in parallel for ultra-fast RL policy training.
- **Fluent C++ SceneBuilder API**: Programmatic 3D scene construction (`.addGroundGrid()`, `.addCube()`, `.addSphere()`, `.addModel()`, `.addJoint()`, `.addURDF()`).
- **Python Gymnasium Interface (`pybind11`)**: Native C++ extension module (`corium_sim_py`) providing standard Gymnasium `CoriumEnv` and `VectorEnv`.

---

## Project Organization

```text
corium-simlab/
├── assets/                          # 3D Mesh Models, Textures, and URDF Files
│   ├── models/                      # sample_robot.obj 3D mesh
│   └── urdf/                        # sample_arm.urdf XML specification
├── include/
│   └── corium_sim/                  # Public C++ Engine Headers
│       ├── kinematics/              # JointKinematics (FK & IK Solvers)
│       ├── physics/                 # 3D Physics & Raycast LiDAR Engine
│       ├── renderer/                # WebGPU Renderer, PBR Materials, SensorCamera, WgslShaders
│       ├── scene/                   # SimScene, SceneBuilder & UrdfLoader
│       ├── App.hpp                  # SimLabApp core application class
│       └── SimLab.hpp               # Single umbrella header inclusion
├── python/
│   └── corium_sim/                  # Python Gymnasium & VectorEnv Package
├── samples/                         # Executable & Script Samples
│   ├── cpp/
│   │   ├── 01_basic_environment.cpp
│   │   └── 02_robotic_arm_manipulator.cpp
│   └── python/
│       ├── 01_gym_environment.py           # Gymnasium step loop
│       ├── 02_robot_joint_control.py       # Robotic joint control & PPM camera frame
│       ├── 03_train_rl_agent.py            # Inverse Kinematics target reach
│       ├── 04_urdf_robot_loader.py         # URDF XML model parsing & control
│       ├── 05_lidar_sensor_scan.py         # 3D LiDAR 360-degree point cloud scan
│       └── 06_vectorized_environments.py  # 8 Parallel vectorized 3D environments
├── src/                             # C++ Core Implementations (.cpp)
│   ├── kinematics/                  # JointKinematics.cpp (FK & IK)
│   ├── physics/                     # PhysicsEngine.cpp & Raycast.cpp
│   ├── python/                      # pybind11 bindings.cpp
│   ├── renderer/                    # WebGpuBackend.cpp & WebGpuSurface.cpp
│   └── scene/                       # SimScene.cpp, SceneBuilder.cpp & UrdfLoader.cpp
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

# Run C++ Industrial Robotic Manipulator Sample
./build/samples/cpp/sample_02_robotic_arm_manipulator
```

### Python Package Installation & CLI Tool

```bash
# Install corium-sim in editable mode
pip install -e . --break-system-packages

# Display system info and WebGPU capabilities
corium-sim info

# Preview any URDF or OBJ 3D model
corium-sim view assets/urdf/sample_arm.urdf

# Run any sample script directly (1 to 6)
corium-sim run 5
```

### Python Gymnasium & Feature Samples

```bash
# Sample #01: Standard Gymnasium Environment Loop
python3 samples/python/01_gym_environment.py

# Sample #02: Joint Kinematics Control & Sensor Image Generation
python3 samples/python/02_robot_joint_control.py

# Sample #03: Inverse Kinematics (IK) Target Tracking
python3 samples/python/03_train_rl_agent.py

# Sample #04: URDF XML Parser & Robot Specification Loader
python3 samples/python/04_urdf_robot_loader.py

# Sample #05: 3D LiDAR 360-degree Point Cloud Scanning
python3 samples/python/05_lidar_sensor_scan.py

# Sample #06: Vectorized Parallel Environments (8 Environments in Parallel)
python3 samples/python/06_vectorized_environments.py
```