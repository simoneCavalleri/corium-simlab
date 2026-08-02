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

    // Build custom 3D Agent Simulation Environment with PBR Materials via Fluent C++ SceneBuilder API
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
                Material::Metallic({0.90f, 0.92f, 0.95f, 1.0f}, 0.20f) // Metallic Chrome Robot
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
            .addPyramid(
                "marker_flag",
                Vec3{-4.0f, 0.0f, -4.0f},
                1.2f,
                2.0f,
                Material::Glossy({0.95f, 0.8f, 0.1f, 1.0f}) // Glossy Gold Marker Flag
            )
            .build();

        app.setScene(std::move(envScene));
    }

    app.run(runtime);
    runtime.shutdown();

    std::cout << "\nCorium SimLab exited successfully.\n";
    return 0;
}
