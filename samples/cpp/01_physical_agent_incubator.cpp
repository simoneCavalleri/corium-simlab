// =============================================================================
// Corium SimLab Sample #01 — Autonomous Mobile Robot (AMR) Navigation & Raycasting
// =============================================================================

#include <iostream>
#include <iomanip>
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
// Agent Goal-Seeking & Obstacle Avoidance Policy (PolicyConcept)
// =============================================================================
class AmrGoalSeekingPolicy {
public:
    AmrGoalSeekingPolicy(float targetX, float targetZ)
        : _targetX(targetX), _targetZ(targetZ) {}

    /// @brief Plan linear and angular movement action based on fused IMU + LiDAR observations.
    template <typename ObservationBuffer>
    [[nodiscard]] inline std::array<float, 2> plan(const ObservationBuffer& fusedObs) noexcept
    {
        std::array<float, 2> action{0.0f, 0.0f};

        float posX = fusedObs[0];
        float posZ = fusedObs[1];
        float minLidarDist = fusedObs[2];

        float dx = _targetX - posX;
        float dz = _targetZ - posZ;
        float distToGoal = std::sqrt(dx * dx + dz * dz);

        // Angle towards target goal
        float targetAngle = std::atan2(dx, dz); // Radians

        if (distToGoal < 0.6f) {
            // Target Reached: Stop agent
            action[0] = 0.0f; // Linear velocity v = 0
            action[1] = 0.0f; // Angular velocity w = 0
        } else if (minLidarDist < 1.5f) {
            // Obstacle Ahead: Slow down and steer away
            action[0] = 0.3f;
            action[1] = 45.0f; // Turn right at 45 deg/sec
        } else {
            // Clear Path: Drive towards goal
            action[0] = std::clamp(distToGoal * 0.8f, 0.4f, 1.8f);
            action[1] = std::clamp(targetAngle * RAD2DEG * 1.5f, -60.0f, 60.0f);
        }

        return action;
    }

private:
    float _targetX = 4.0f;
    float _targetZ = -2.0f;
};

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — Sample #01: Physical AMR Navigation & Obstacle Avoidance\n";
    std::cout << " Features: 3D WebGPU Window | Real-Time LiDAR Raycasting | Goal-Seeking Policy\n";
    std::cout << "=========================================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AGENT SPECIFICATION BLUEPRINT (7 Agent-Centric Pillars)
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Decoupled AgentSpec Blueprint...\n";

    auto amrAgentSpec = makeAgentSpec()
        // Pillar 2: Agent Body Model
        .withModel(SimEntity{
            .name = "amr_robot",
            .position = {0.0f, 0.5f, 0.0f},
            .mass = 15.0f
        })
        // Pillar 3: Onboard Sensors (IMU + 180-Degree Custom LiDAR Raycaster)
        .withSensors(
            sensors::ImuSensor{},
            sensors::makeCustomSensor<180>([](const SimEntity& agent, const SimScene& scene) {
                std::array<float, 180> scan{};
                float yawRad = agent.rotation.y * DEG2RAD;
                Vec3 origin = agent.position + Vec3{0.0f, 0.5f, 0.0f};

                for (int i = 0; i < 180; ++i) {
                    float angleDeg = static_cast<float>(i - 90);
                    float rayAngle = yawRad + angleDeg * DEG2RAD;
                    Vec3 dir{std::sin(rayAngle), 0.0f, std::cos(rayAngle)};

                    auto hit = Raycast::castRay(scene, origin, dir, 20.0f);
                    scan[i] = hit.hit ? hit.distance : 20.0f;
                }
                return scan;
            })
        )
        // Pillar 4: Perception Chain (Fuse IMU PosX, PosZ, and Minimum LiDAR obstacle clearance)
        .withPerceptionChain<3>([](const auto& rawObs) {
            std::array<float, 3> fused{};
            fused[0] = rawObs[0]; // IMU PosX
            fused[1] = rawObs[2]; // IMU PosZ

            float minLidarDist = 20.0f;
            for (std::size_t i = 12; i < 12 + 180; ++i) {
                minLidarDist = std::min(minLidarDist, rawObs[i]);
            }
            fused[2] = minLidarDist;
            return fused;
        })
        // Pillar 5: Actuators (Differential Mobile Drive: v, w)
        .withActuators(
            actuators::DifferentialDriveActuator{}
        )
        // Pillar 6: Planning Policy
        .withPolicy(AmrGoalSeekingPolicy{4.0f, -2.0f});

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT 3D WEBGPU VISUAL INCUBATOR SCENE & SPAWN AGENT
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing 3D WebGPU Visual Scene & Spawning Agent...\n";

    auto incubator = makeIncubator<KinematicPhysicsEngine>()
        // Pillar 1: Environment & 3D Visual Scene Setup
        .withEnvironment([](SimScene& scene) {
            scene.addEntity(SimEntity{.name = "user_ground", .shape = EntityShape::PlaneGrid, .isStatic = true});
            scene.addEntity(SimEntity{
                .name = "target_goal",
                .material = Material::Metallic({0.95f, 0.15f, 0.15f, 1.0f}, 0.20f),
                .position = {4.0f, 0.75f, -2.0f},
                .scale = {1.2f, 1.2f, 1.2f},
                .isStatic = true
            });
            scene.addEntity(SimEntity{
                .name = "safety_barrier",
                .material = Material::Matte({0.95f, 0.80f, 0.05f, 1.0f}),
                .position = {-2.5f, 0.6f, 2.5f},
                .scale = {3.5f, 1.2f, 0.2f},
                .isStatic = true
            });
        })
        .spawnAgent("amr_leader", std::move(amrAgentSpec), Vec3{0.0f, 0.5f, 0.0f})
        // Pillar 7: Reward Policy
        .withRewardPolicy(
            RewardBuilder{}
                .withTarget({4.0f, 0.75f, -2.0f})
                .withGoalThreshold(0.6f)
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<CollisionPenalty>(50.0f)
                .addTerm<GoalReachedBonus>(200.0f, 0.6f)
        );

    auto incubatorPtr = std::make_shared<decltype(incubator)>(std::move(incubator));

    // Connect shared 3D scene to generic WebGPU App
    app.setScene(incubatorPtr->scenePtr());

    // Register high-frequency step callback for real-time WebGPU rendering & active agent navigation
    app.onStep([incubatorPtr](SimLabApp&, float dt) {
        auto result = incubatorPtr->step(dt);
        (void)result;
    });

    std::cout << "  - 3D WebGPU Visual Environment initialized successfully!\n";
    std::cout << "  - Interactive 3D Window: Orbit/Pan Camera with WASD & Mouse!\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN 3D INTERACTIVE WEBGPU RENDERING & SIMULATION LOOP
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Launching 3D Interactive WebGPU Graphics Window...\n";
    app.run(runtime);

    runtime.shutdown();
    std::cout << "\n[Incubator] 3D Visual Physical Agent Incubator application shutdown complete.\n";
    return 0;
}
