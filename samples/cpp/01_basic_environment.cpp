// =============================================================================
// Corium SimLab Sample #01 — Basic 3D PBR Simulation Environment Scene
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
    std::cout << " Corium SimLab Sample #01: Basic 3D PBR Environment\n";
    std::cout << "=========================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    WGPUDevice device = app.renderer().device();
    WGPUQueue queue = app.renderer().queue();

    if (device && queue) {
        auto envScene = scene::SimScene::builder(device, queue)
            .addGroundGrid(50.0f, 50.0f, 50)
            .addCube(
                "agent_robot",
                Vec3{0.0f, 0.5f, 0.0f},
                Vec3{1.0f, 1.0f, 1.0f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Metallic({0.90f, 0.92f, 0.95f, 1.0f}, 0.20f)
            )
            .addCube(
                "target_goal",
                Vec3{4.0f, 0.75f, -2.0f},
                Vec3{1.5f, 1.5f, 1.5f},
                Vec3{0.0f, 0.0f, 0.0f},
                Material::Glossy({1.0f, 0.2f, 0.15f, 1.0f})
            )
            .addSphere(
                "obstacle_a",
                Vec3{-3.0f, 1.0f, 3.0f},
                1.0f,
                Vec3{1.0f, 1.0f, 1.0f},
                Material::Matte({0.2f, 0.7f, 0.9f, 1.0f})
            )
            .build();

        app.setScene(std::move(envScene));
    }

    app.run(runtime);
    runtime.shutdown();
    return 0;
}
