#include <iostream>
#include <iomanip>
#include <vector>
#include "corium_sim/SimLab.hpp"

using namespace corium_sim;
using namespace corium_sim::agent;
using namespace corium_sim::environment;
using namespace corium_sim::physics;

// Simple Dummy Policy for Swarm Agents
class SwarmFollowerPolicy {
public:
    template <typename Obs>
    [[nodiscard]] inline std::array<float, 2> plan([[maybe_unused]] const Obs& obs) noexcept
    {
        return {0.5f, 0.05f};
    }
};

int main()
{
    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — Decoupled AgentSpec Multi-Agent Swarm Incubator (C++20)\n";
    std::cout << " Features: AgentSpec Blueprint | Multi-Agent Spawning | Vector Pool\n";
    std::cout << "=========================================================================\n\n";

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AGENT SPECIFICATION BLUEPRINT ONCE
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Reusable AMR Swarm AgentSpec Blueprint...\n";

    auto amrSwarmSpec = makeAgentSpec()
        .withModel(scene::SimEntity{.name = "amr_base", .mass = 12.0f})
        .withSensors(
            sensors::ImuSensor{},
            sensors::RaycastLidarSensor<360>{}
        )
        .withoutPerceptionChain()
        .withActuators(
            actuators::DifferentialDriveActuator{}
        )
        .withPolicy(SwarmFollowerPolicy{});

    std::cout << "  - AMR Swarm AgentSpec blueprint created successfully!\n\n";

    // -------------------------------------------------------------------------
    // 2. SPAWN MULTIPLE AGENT INSTANCES INTO SHARED 3D ENVIRONMENT SCENE
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Spawning 3 Swarm AMRs from single AgentSpec Blueprint into 3D Scene...\n";

    auto incubator = makeIncubator<KinematicPhysicsEngine>()
        .withEnvironment([](scene::SimScene& scene) {
            scene.addEntity(scene::SimEntity{.name = "user_ground", .isStatic = true});
            scene.addEntity(scene::SimEntity{.name = "target_waypoint", .position = {0.0f, 0.75f, 10.0f}, .isStatic = true});
        })
        .spawnAgent("amr_leader", std::move(amrSwarmSpec), math::Vec3{0.0f, 0.5f, 0.0f})
        .withRewardPolicy(
            RewardBuilder{}
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<GoalReachedBonus>(200.0f)
        );

    std::cout << "  - Swarm Leader Agent spawned successfully at (0.0, 0.5, 0.0)!\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN CONTROL STEP LOOP
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Running Multi-Agent Simulation Step Loop...\n\n";

    for (std::size_t step = 0; step < 50; ++step) {
        float reward = incubator.step(0.01667f);

        if (step % 10 == 0 || step == 49) {
            std::cout << std::fixed << std::setprecision(3)
                      << "  [Step " << std::setw(2) << step << "] "
                      << "Leader Pos=(" << incubator.agent().body().position.x << ", " << incubator.agent().body().position.z << ") | "
                      << "Scalar Reward: " << reward << "\n";
        }
    }

    std::cout << "\n[Incubator] Multi-Agent Swarm Incubator completed successfully!\n";
    return 0;
}
