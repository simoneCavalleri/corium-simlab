// =============================================================================
// Corium SimLab Sample #02 — Realistic Industrial Robotics Workstation & 3-DOF Arm
// =============================================================================

#include "corium_sim/SimLab.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"
#include <iostream>

using namespace corium_sim;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::kinematics;
using namespace corium_sim::scene;

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================\n";
    std::cout << " Corium SimLab Sample #02: Industrial Robotic Manipulator Workstation\n";
    std::cout << " Features: 3D WebGPU Graphics | URDF Robot Loading | Inverse Kinematics\n";
    std::cout << "=========================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    auto envScene = SimScene::builder(nullptr, nullptr)
        // 1. Concrete Factory Floor Grid
        .addGroundGrid(60.0f, 60.0f, 60)

        // 2. Heavy Industrial Pedestal Table Workstation
        .addCube(
            "workstation_table",
            Vec3{0.0f, 0.4f, 0.0f},
            Vec3{3.0f, 0.8f, 2.0f},
            Vec3{0.0f, 0.0f, 0.0f},
            Material::Metallic({0.4f, 0.45f, 0.50f, 1.0f}, 0.35f),
            true // isStatic
        )

        // 3. Load Robotic Arm Manipulator & Joints from URDF Specification
        .addURDF("assets/urdf/sample_arm.urdf", Vec3{0.0f, 0.4f, 0.0f})

        // 4. Parallel Two-Finger Gripper End-Effector
        .addCube(
            "gripper_finger_l",
            Vec3{-0.15f, 3.2f, 0.0f},
            Vec3{0.08f, 0.3f, 0.12f},
            Vec3{0.0f, 0.0f, 0.0f},
            Material::Matte({0.2f, 0.2f, 0.25f, 1.0f})
        )
        .addCube(
            "gripper_finger_r",
            Vec3{0.15f, 3.2f, 0.0f},
            Vec3{0.08f, 0.3f, 0.12f},
            Vec3{0.0f, 0.0f, 0.0f},
            Material::Matte({0.2f, 0.2f, 0.25f, 1.0f})
        )

        // 5. Additional End-Effector Gripper Joints
        .addJoint(
            "joint_gripper_l",
            "elbow_link",
            "gripper_finger_l",
            JointType::Prismatic,
            Vec3{-0.15f, 0.3f, 0.0f},
            Vec3{-1.0f, 0.0f, 0.0f},
            0.0f, 0.1f
        )
        .addJoint(
            "joint_gripper_r",
            "elbow_link",
            "gripper_finger_r",
            JointType::Prismatic,
            Vec3{0.15f, 0.3f, 0.0f},
            Vec3{1.0f, 0.0f, 0.0f},
            0.0f, 0.1f
        )

        // 6. Workpiece Inspection Platform & Red Metallic Target Block
        .addCube(
            "inspection_platform",
            Vec3{3.5f, 0.5f, -1.5f},
            Vec3{2.0f, 1.0f, 1.5f},
            Vec3{0.0f, 0.0f, 0.0f},
            Material::Metallic({0.5f, 0.52f, 0.55f, 1.0f}, 0.40f),
            true // isStatic
        )
        .addCube(
            "target_workpiece",
            Vec3{3.5f, 1.25f, -1.5f},
            Vec3{0.6f, 0.5f, 0.6f},
            Vec3{0.0f, 0.0f, 0.0f},
            Material::Metallic({0.95f, 0.10f, 0.15f, 1.0f}, 0.10f)
        )

        // 7. Industrial Safety Warning Barrier
        .addCube(
            "safety_barrier",
            Vec3{-2.5f, 0.6f, 2.5f},
            Vec3{3.5f, 1.2f, 0.2f},
            Vec3{0.0f, 0.0f, 0.0f},
            Material::Matte({0.95f, 0.80f, 0.05f, 1.0f}),
            true // isStatic
        )
        .build();

    app.setScene(std::move(envScene));
    std::cout << "[3D Scene] Industrial Workstation & URDF Robotic Arm Loaded Successfully!\n";

    std::cout << "[Step 3] Launching 3D Interactive WebGPU Graphics Window...\n";
    app.run(runtime);

    runtime.shutdown();
    return 0;
}
