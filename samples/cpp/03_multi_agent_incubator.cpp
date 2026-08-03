// =============================================================================
// Corium SimLab Sample #03 — Multi-Agent Warehouse Logistics Swarm Navigation
// =============================================================================

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "corium_sim/SimLab.hpp"

using namespace corium_sim;
using namespace corium_sim::agent;
using namespace corium_sim::environment;
using namespace corium_sim::physics;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::scene;

// =============================================================================
// Swarm Leader Navigation Policy (PolicyConcept)
// =============================================================================
class SwarmLeaderPolicy {
public:
    SwarmLeaderPolicy(float targetX, float targetZ)
        : _targetX(targetX), _targetZ(targetZ) {}

    template <typename Obs>
    [[nodiscard]] inline std::array<float, 2> plan(const Obs& obs) noexcept
    {
        std::array<float, 2> action{0.0f, 0.0f};
        float posX = obs[0];
        float posZ = obs[2];

        float dx = _targetX - posX;
        float dz = _targetZ - posZ;
        float dist = std::sqrt(dx * dx + dz * dz);

        if (dist > 0.5f) {
            action[0] = std::clamp(dist * 0.5f, 0.4f, 1.5f); // Linear velocity v
            action[1] = std::clamp(std::atan2(dx, dz) * RAD2DEG * 1.2f, -40.0f, 40.0f); // Angular velocity w
        }

        return action;
    }

private:
    float _targetX = 0.0f;
    float _targetZ = 12.0f;
};

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — Sample #03: Multi-Agent Warehouse Logistics Swarm\n";
    std::cout << " Features: Swarm Navigation | Formation Control | 3D WebGPU Visuals\n";
    std::cout << "=========================================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AMR SWARM AGENT BLUEPRINT
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Reusable Swarm AMR AgentSpec Blueprint...\n";

    auto amrSwarmSpec = makeAgentSpec()
        .withModel(SimEntity{.name = "amr_leader", .mass = 12.0f})
        .withSensors(
            sensors::ImuSensor{}
        )
        .withoutPerceptionChain()
        .withActuators(
            actuators::DifferentialDriveActuator{}
        )
        .withPolicy(SwarmLeaderPolicy{0.0f, 12.0f});

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT 3D SCENE & SPAWN SWARM
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing 3D Scene & Spawning Swarm AMRs...\n";

    auto incubator = makeIncubator<KinematicPhysicsEngine>()
        .withEnvironment([](SimScene& scene) {
            // Warehouse Floor Grid
            scene.addEntity(SimEntity{.name = "user_ground", .shape = EntityShape::PlaneGrid, .isStatic = true});

            // Follower AMR 1 (Left Wing)
            scene.addEntity(SimEntity{
                .name = "amr_follower_1",
                .material = Material::Metallic({0.2f, 0.6f, 0.9f, 1.0f}, 0.2f),
                .position = {-2.5f, 0.5f, -2.0f},
                .scale = {1.0f, 0.8f, 1.0f}
            });

            // Follower AMR 2 (Right Wing)
            scene.addEntity(SimEntity{
                .name = "amr_follower_2",
                .material = Material::Metallic({0.2f, 0.6f, 0.9f, 1.0f}, 0.2f),
                .position = {2.5f, 0.5f, -2.0f},
                .scale = {1.0f, 0.8f, 1.0f}
            });

            // Warehouse Target Waypoint Station
            scene.addEntity(SimEntity{
                .name = "target_waypoint",
                .material = Material::Metallic({0.1f, 0.9f, 0.2f, 1.0f}, 0.1f),
                .position = {0.0f, 0.75f, 12.0f},
                .scale = {1.5f, 1.5f, 1.5f},
                .isStatic = true
            });
        })
        .spawnAgent("amr_leader", std::move(amrSwarmSpec), Vec3{0.0f, 0.5f, 0.0f})
        .withRewardPolicy(
            RewardBuilder{}
                .withTarget({0.0f, 0.75f, 12.0f})
                .withGoalThreshold(0.5f)
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<GoalReachedBonus>(200.0f, 0.5f)
        );

    auto incubatorPtr = std::make_shared<decltype(incubator)>(std::move(incubator));

    // Connect shared 3D scene to generic WebGPU App
    app.setScene(incubatorPtr->scenePtr());

    // Step callback updating leader navigation and follower formation control
    app.onStep([incubatorPtr](SimLabApp& app, float dt) {
        auto result = incubatorPtr->step(dt);
        (void)result;

        // Leader AMR position
        Vec3 leaderPos{0.0f, 0.5f, 0.0f};
        if (auto* leader = app.scene().findEntity("amr_leader")) {
            leaderPos = leader->position;
        }

        // Maintain V-shape formation for follower AMRs
        if (auto* follower1 = app.scene().findEntity("amr_follower_1")) {
            Vec3 targetFollower1Pos = leaderPos + Vec3{-2.5f, 0.0f, -2.0f};
            follower1->position.x += (targetFollower1Pos.x - follower1->position.x) * 3.0f * dt;
            follower1->position.z += (targetFollower1Pos.z - follower1->position.z) * 3.0f * dt;
        }
        if (auto* follower2 = app.scene().findEntity("amr_follower_2")) {
            Vec3 targetFollower2Pos = leaderPos + Vec3{2.5f, 0.0f, -2.0f};
            follower2->position.x += (targetFollower2Pos.x - follower2->position.x) * 3.0f * dt;
            follower2->position.z += (targetFollower2Pos.z - follower2->position.z) * 3.0f * dt;
        }
    });

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
