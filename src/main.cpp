// =============================================================================
// Corium SimLab — Real-Time Agent Simulation Engine & WebGPU Visualizer
//
// Standalone Application Entry Point & Fluent C++ Environment Builder Exemplar
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
    std::cout << " Corium SimLab Engine v1.2.0 (Fluent C++ Simulation Core)\n";
    std::cout << "=========================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    // Build custom 3D Agent Simulation Environment with Articulated Robot Arm Joints
    WGPUDevice device = app.renderer().device();
    WGPUQueue queue = app.renderer().queue();

    if (device && queue) {
        auto envScene = scene::SimScene::builder(device, queue)
            .addGroundGrid(60.0f, 60.0f, 60)
            .addModel(
                "agent_robot",
                "assets/models/sample_robot.obj",
                Vec3{0.0f, 0.0f, 0.0f},
                Vec3{1.0f, 1.0f, 1.0f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Metallic({0.90f, 0.92f, 0.95f, 1.0f}, 0.20f) // Metallic Chrome Base Robot
            )
            .addCube(
                "arm_link1",
                Vec3{0.0f, 1.2f, 0.0f},
                Vec3{0.4f, 1.5f, 0.4f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Glossy({0.2f, 0.8f, 0.3f, 1.0f}) // Green Upper Arm Link
            )
            .addCube(
                "arm_link2",
                Vec3{0.0f, 2.5f, 0.0f},
                Vec3{0.3f, 1.2f, 0.3f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Glossy({0.9f, 0.5f, 0.1f, 1.0f}) // Orange Forearm Link
            )
            .addJoint(
                "joint_shoulder",
                "agent_robot",
                "arm_link1",
                JointType::Revolute,
                Vec3{0.0f, 1.0f, 0.0f},  // Anchor point at top of base
                Vec3{0.0f, 1.0f, 0.0f},  // Yaw rotation axis
                -3.14159f, 3.14159f
            )
            .addJoint(
                "joint_elbow",
                "arm_link1",
                "arm_link2",
                JointType::Revolute,
                Vec3{0.0f, 1.5f, 0.0f},  // Anchor point at elbow
                Vec3{1.0f, 0.0f, 0.0f},  // Pitch rotation axis
                -1.5708f, 1.5708f
            )
            .addCube(
                "target_goal",
                Vec3{4.0f, 0.75f, -2.0f},
                Vec3{1.5f, 1.5f, 1.5f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Glossy({1.0f, 0.2f, 0.15f, 1.0f}) // Glossy Red Goal Box
            )
            .addSphere(
                "obstacle_a",
                Vec3{-3.0f, 1.0f, 3.0f},
                1.0f,
                Vec3{1.0f, 1.0f, 1.0f},
                Material::Matte({0.2f, 0.7f, 0.9f, 1.0f}) // Matte Cyan Obstacle
            )
            .build();

        app.setScene(std::move(envScene));
    }

    app.run(runtime);
    runtime.shutdown();

    std::cout << "\nCorium SimLab exited successfully.\n";
    return 0;
}
