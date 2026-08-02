// =============================================================================
// Corium SimLab Sample #02 — Realistic Industrial Robotics Workstation & 3-DOF Arm
// =============================================================================

#include "corium_sim/App.hpp"
#include <iostream>

using namespace corium_sim;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::kinematics;

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================\n";
    std::cout << " Corium SimLab Sample #02: Industrial Robotic Manipulator Workstation\n";
    std::cout << "=========================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    WGPUDevice device = app.renderer().device();
    WGPUQueue queue = app.renderer().queue();

    if (device && queue) {
        auto envScene = scene::SimScene::builder(device, queue)
            // 1. Concrete Factory Floor Grid
            .addGroundGrid(60.0f, 60.0f, 60)

            // 2. Heavy Industrial Pedestal Table Workstation
            .addCube(
                "workstation_table",
                Vec3{0.0f, 0.4f, 0.0f},
                Vec3{3.0f, 0.8f, 2.0f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Metallic({0.4f, 0.45f, 0.50f, 1.0f}, 0.35f), // Cast Iron Table Base
                true // isStatic
            )

            // 3. Robotic Manipulator Pedestal Base
            .addModel(
                "agent_robot",
                "assets/models/sample_robot.obj",
                Vec3{0.0f, 0.8f, 0.0f},
                Vec3{0.8f, 0.8f, 0.8f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Metallic({0.95f, 0.95f, 0.98f, 1.0f}, 0.15f), // Chrome Base Pedestal
                true // isStatic
            )

            // 4. Robotic Arm Links
            .addCube(
                "shoulder_link",
                Vec3{0.0f, 1.4f, 0.0f},
                Vec3{0.4f, 1.0f, 0.4f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Glossy({0.15f, 0.55f, 0.95f, 1.0f}) // Industrial Blue Upper Arm
            )
            .addCube(
                "elbow_link",
                Vec3{0.0f, 2.3f, 0.0f},
                Vec3{0.3f, 0.8f, 0.3f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Glossy({0.95f, 0.85f, 0.10f, 1.0f}) // Safety Yellow Forearm
            )
            .addCube(
                "wrist_link",
                Vec3{0.0f, 2.9f, 0.0f},
                Vec3{0.25f, 0.4f, 0.25f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Metallic({0.85f, 0.88f, 0.90f, 1.0f}, 0.25f) // Wrist Metallic Housing
            )

            // 5. Parallel Two-Finger Gripper End-Effector
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

            // 6. Articulated Kinematic Joints
            .addJoint(
                "joint_shoulder_yaw",
                "agent_robot",
                "shoulder_link",
                JointType::Revolute,
                Vec3{0.0f, 0.6f, 0.0f},  // Yaw rotation around vertical Z/Y
                Vec3{0.0f, 1.0f, 0.0f},
                -3.14159f, 3.14159f
            )
            .addJoint(
                "joint_elbow_pitch",
                "shoulder_link",
                "elbow_link",
                JointType::Revolute,
                Vec3{0.0f, 0.9f, 0.0f},  // Pitch rotation around X axis
                Vec3{1.0f, 0.0f, 0.0f},
                -2.0944f, 2.0944f
            )
            .addJoint(
                "joint_wrist_roll",
                "elbow_link",
                "wrist_link",
                JointType::Revolute,
                Vec3{0.0f, 0.6f, 0.0f},  // Wrist roll around Y axis
                Vec3{0.0f, 1.0f, 0.0f},
                -3.14159f, 3.14159f
            )
            .addJoint(
                "joint_gripper_l",
                "wrist_link",
                "gripper_finger_l",
                JointType::Prismatic,
                Vec3{-0.15f, 0.3f, 0.0f}, // Linear slide inwards/outwards
                Vec3{-1.0f, 0.0f, 0.0f},
                0.0f, 0.1f
            )
            .addJoint(
                "joint_gripper_r",
                "wrist_link",
                "gripper_finger_r",
                JointType::Prismatic,
                Vec3{0.15f, 0.3f, 0.0f},
                Vec3{1.0f, 0.0f, 0.0f},
                0.0f, 0.1f
            )

            // 7. Workpiece Inspection Platform & Red Metallic Target Block
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
                Material::Metallic({0.95f, 0.10f, 0.15f, 1.0f}, 0.10f) // Anodized Red Metal Workpiece Target
            )

            // 8. Industrial Safety Warning Barrier
            .addCube(
                "safety_barrier",
                Vec3{-2.5f, 0.6f, 2.5f},
                Vec3{3.5f, 1.2f, 0.2f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Matte({0.95f, 0.80f, 0.05f, 1.0f}), // High-Contrast Safety Yellow
                true // isStatic
            )
            .build();

        app.setScene(std::move(envScene));
    }

    app.run(runtime);
    runtime.shutdown();
    return 0;
}
