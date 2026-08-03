#include <iostream>
#include <iomanip>
#include <cmath>
#include "corium_sim/SimLab.hpp"

using namespace corium_sim;
using namespace corium_sim::agent;
using namespace corium_sim::environment;
using namespace corium_sim::physics;

// =============================================================================
// User Planning Policy implementing PolicyConcept
// =============================================================================
class UserPlannerPolicy {
public:
    UserPlannerPolicy(float targetX, float targetZ)
        : _targetX(targetX), _targetZ(targetZ) {}

    template <typename ObservationBuffer>
    [[nodiscard]] inline auto plan(const ObservationBuffer& fusedObs) noexcept
    {
        std::array<float, 8> action{};
        float posX = fusedObs[0];
        float posZ = fusedObs[1];

        float dx = _targetX - posX;
        float dz = _targetZ - posZ;
        float v = std::clamp(dz * 0.5f, -1.0f, 1.0f);
        float w = std::clamp(dx * 15.0f, -45.0f, 45.0f);

        for (std::size_t j = 0; j < 6; ++j) {
            action[j] = 0.1f * static_cast<float>(j);
        }
        action[6] = v;
        action[7] = w;

        return action;
    }

private:
    float _targetX = 4.0f;
    float _targetZ = -2.0f;
};

int main()
{
    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — Decoupled AgentSpec & Physical Agent Incubator (C++20)\n";
    std::cout << " Features: AgentSpec Blueprint | Environment Spawning | Composable Rewards\n";
    std::cout << "=========================================================================\n\n";

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AGENT SPECIFICATION (Model, Sensors, Perception, Actuators, Policy)
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Decoupled AgentSpec Blueprint...\n";

    auto amrAgentSpec = makeAgentSpec()
        // A. Agent Model Entity
        .withModel(scene::SimEntity{.name = "amr_robot", .mass = 15.0f})
        // B. Multi-Modal Sensors (Preset + Custom 180-deg LiDAR Raycasting)
        .withSensors(
            sensors::ImuSensor{},
            sensors::JointEncoderSensor<6>{},
            sensors::makeCustomSensor<180>([](const scene::SimEntity& agent, const scene::SimScene& scene) {
                std::array<float, 180> scan{};
                float yawRad = agent.rotation.y * math::DEG2RAD;
                math::Vec3 origin = agent.position + math::Vec3{0.0f, 0.5f, 0.0f};

                for (int i = 0; i < 180; ++i) {
                    float angleDeg = static_cast<float>(i - 90);
                    float rayAngle = yawRad + angleDeg * math::DEG2RAD;
                    math::Vec3 dir{std::sin(rayAngle), 0.0f, std::cos(rayAngle)};

                    auto hit = Raycast::castRay(scene, origin, dir, 20.0f);
                    scan[i] = hit.hit ? hit.distance : 20.0f;
                }
                return scan;
            })
        )
        // C. Perception Processing & Sensor Fusion Chain
        .withPerceptionChain<3>([](const auto& rawObs) {
            std::array<float, 3> fused{};
            fused[0] = rawObs[0]; // IMU PosX
            fused[1] = rawObs[2]; // IMU PosZ

            float minLidarDist = 20.0f;
            for (std::size_t i = 21; i < 21 + 180; ++i) {
                minLidarDist = std::min(minLidarDist, rawObs[i]);
            }
            fused[2] = minLidarDist;
            return fused;
        })
        // D. Physical Actuators Suite
        .withActuators(
            actuators::JointPositionActuator<6>{},
            actuators::DifferentialDriveActuator{}
        )
        // E. Planning Policy / Brain Algorithm
        .withPolicy(UserPlannerPolicy{4.0f, -2.0f});

    std::cout << "  - AgentSpec blueprint created successfully!\n\n";

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT INCUBATOR ENVIRONMENT & SPAWN AGENT FROM AGENTSPEC
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing Environment & Spawning Agent from AgentSpec...\n";

    auto incubator = makeIncubator<KinematicPhysicsEngine>()
        .withEnvironment([](scene::SimScene& scene) {
            scene.addEntity(scene::SimEntity{.name = "user_ground", .isStatic = true});
            scene.addEntity(scene::SimEntity{.name = "target_goal", .position = {4.0f, 0.75f, -2.0f}, .isStatic = true});
        })
        .spawnAgent("amr_leader", std::move(amrAgentSpec), math::Vec3{0.0f, 0.5f, 0.0f})

        .withRewardPolicy(
            RewardBuilder{}
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<CollisionPenalty>(50.0f)
                .addTerm<GoalReachedBonus>(200.0f)
        );

    std::cout << "  - Incubator environment initialized & agent spawned successfully!\n";
    std::cout << "  - Fused Observation Dimension (dim O): " << decltype(incubator)::AgentType::observation_size << " floats\n";
    std::cout << "  - Action Space Dimension       (dim A): " << decltype(incubator)::AgentType::action_size << " floats\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN HIGH-FREQUENCY SIMULATION STEP LOOP
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Running Control Loop (Perception -> Policy -> Action -> Physics -> Reward)...\n\n";

    for (std::size_t step = 0; step < 100; ++step) {
        // Step coordinated incubator: perception -> policy -> action -> physics -> reward
        float reward = incubator.step(0.01667f);

        // Update body pose
        incubator.agent().body().position.z += incubator.agent().body().velocity.z * 0.01667f;
        incubator.agent().body().rotation.y += incubator.agent().body().angularVelocity.y * 0.01667f;

        if (step % 20 == 0 || step == 99) {
            auto fusedObs = incubator.agent().observe(incubator.env().scene());
            std::cout << std::fixed << std::setprecision(3)
                      << "  [Step " << std::setw(2) << step << "] "
                      << "Agent Pos=(" << incubator.agent().body().position.x << ", " << incubator.agent().body().position.z << ") | "
                      << "Reward: " << std::setw(7) << reward << " | "
                      << "Fused Obs=[PosX: " << fusedObs[0] << ", PosZ: " << fusedObs[1] << ", MinLidarDist: " << fusedObs[2] << " m]\n";
        }
    }

    std::cout << "\n[Incubator] Decoupled AgentSpec simulation completed with ZERO heap allocations!\n";
    return 0;
}
