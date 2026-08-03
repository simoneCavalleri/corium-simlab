#include <iostream>
#include <iomanip>
#include <vector>
#include "corium_sim/SimLab.hpp"

using namespace corium_sim;
using namespace corium_sim::agent;
using namespace corium_sim::environment;
using namespace corium_sim::physics;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::scene;

// Simple Dummy Policy for Swarm Agents
class SwarmFollowerPolicy {
public:
    template <typename Obs>
    [[nodiscard]] inline std::array<float, 2> plan([[maybe_unused]] const Obs& obs) noexcept
    {
        return {0.5f, 0.05f};
    }
};

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — 3D WebGPU Multi-Agent Swarm Incubator\n";
    std::cout << " Features: 3D Swarm Navigation | Sim-to-Real Randomization | 60 FPS Visuals\n";
    std::cout << "=========================================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AGENT SPECIFICATION BLUEPRINT ONCE
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Reusable AMR Swarm AgentSpec Blueprint...\n";

    auto amrSwarmSpec = makeAgentSpec()
        .withModel(SimEntity{.name = "amr_base", .mass = 12.0f})
        .withSensors(
            sensors::ImuSensor{},
            sensors::RaycastLidarSensor<360>{}
        )
        .withoutPerceptionChain()
        .withActuators(
            actuators::DifferentialDriveActuator{}
        )
        .withPolicy(SwarmFollowerPolicy{});

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT 3D WEBGPU MULTI-AGENT SCENE & SPAWN SWARM
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing 3D Scene & Spawning Swarm AMRs...\n";

    auto incubator = makeIncubator<KinematicPhysicsEngine>()
        .withEnvironment([](SimScene& scene) {
            scene.addEntity(SimEntity{.name = "user_ground", .isStatic = true});
            scene.addEntity(SimEntity{
                .name = "amr_follower_1",
                .material = Material::Metallic({0.2f, 0.6f, 0.9f, 1.0f}, 0.2f),
                .position = {-2.0f, 0.5f, -1.0f},
                .scale = {1.0f, 0.8f, 1.0f}
            });
            scene.addEntity(SimEntity{
                .name = "amr_follower_2",
                .material = Material::Metallic({0.2f, 0.6f, 0.9f, 1.0f}, 0.2f),
                .position = {2.0f, 0.5f, -1.0f},
                .scale = {1.0f, 0.8f, 1.0f}
            });
            scene.addEntity(SimEntity{
                .name = "target_waypoint",
                .material = Material::Metallic({0.1f, 0.9f, 0.2f, 1.0f}, 0.1f),
                .position = {0.0f, 0.75f, 10.0f},
                .scale = {1.5f, 1.5f, 1.5f},
                .isStatic = true
            });
        })
        .spawnAgent("amr_leader", std::move(amrSwarmSpec), Vec3{0.0f, 0.5f, 0.0f})
        .withRewardPolicy(
            RewardBuilder{}
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<GoalReachedBonus>(200.0f)
        );

    auto incubatorPtr = std::make_shared<decltype(incubator)>(std::move(incubator));

    app.onStep([incubatorPtr](SimLabApp& appInstance, float dt) {
        incubatorPtr->step(dt);

        if (auto* follower1 = appInstance.scene().findEntity("amr_follower_1")) {
            follower1->velocity.z = 0.6f;
            follower1->rotation.y += 10.0f * dt;
        }
        if (auto* follower2 = appInstance.scene().findEntity("amr_follower_2")) {
            follower2->velocity.z = 0.8f;
            follower2->rotation.y -= 15.0f * dt;
        }
    });

    app.setScene(incubatorPtr->scenePtr());


    std::cout << "  - 3D Multi-Agent Swarm Environment initialized successfully!\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN 3D INTERACTIVE WEBGPU RENDERING & SIMULATION LOOP
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Launching 3D Interactive WebGPU Graphics Window...\n";
    app.run(runtime);

    runtime.shutdown();
    std::cout << "\n[Incubator] 3D Multi-Agent Swarm Incubator application shutdown complete.\n";
    return 0;
}
