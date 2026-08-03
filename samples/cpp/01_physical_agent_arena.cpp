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
/// @brief Goal-seeking policy for differential-drive AMR.
/// Uses posX, posZ, yaw, and minimum LiDAR clearance to navigate to goal
/// while avoiding obstacles.
class AmrGoalSeekingPolicy {
public:
    AmrGoalSeekingPolicy(float targetX, float targetZ)
        : _targetX(targetX), _targetZ(targetZ) {}

    /// @brief Plan linear/angular velocity from [posX, posZ, yawDeg, minLidar] observation.
    template <typename ObservationBuffer>
    [[nodiscard]] inline std::array<float, 2> plan(const ObservationBuffer& obs) noexcept
    {
        std::array<float, 2> action{0.0f, 0.0f};

        float posX        = obs[0]; // World X position
        float posZ        = obs[1]; // World Z position
        float agentYaw    = obs[2]; // Agent heading (degrees, world frame)
        float minLidar    = obs[3]; // Minimum obstacle clearance (meters)

        float dx = _targetX - posX;
        float dz = _targetZ - posZ;
        float dist = std::sqrt(dx * dx + dz * dz);

        if (dist < 0.6f) {
            // Goal reached — stop
            return {0.0f, 0.0f};
        }

        // Desired heading toward goal (world frame, degrees)
        float desiredYaw = std::atan2(dx, dz) * RAD2DEG;

        // Relative heading error [-180, +180] degrees
        float angleError = desiredYaw - agentYaw;
        while (angleError >  180.0f) angleError -= 360.0f;
        while (angleError < -180.0f) angleError += 360.0f;

        if (minLidar < 1.5f) {
            // Obstacle ahead: slow down and steer away
            action[0] = 0.3f;
            action[1] = 45.0f;
        } else {
            // Clear path: drive toward goal
            action[0] = std::clamp(dist * 0.8f, 0.4f, 1.8f);
            action[1] = std::clamp(angleError * 1.5f, -60.0f, 60.0f);
        }

        return action;
    }

private:
    float _targetX;
    float _targetZ;
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

    events::SimEventTracer tracer;
    tracer.registerWith(app);

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
        // Pillar 3: Onboard Sensors (IMU + 180-Ray Real-Time Visualized LiDAR)
        .withSensors(
            sensors::ImuSensor{},
            sensors::LidarSensor<180>{20.0f, 180.0f, true}
        )
        // Pillar 4: Perception Chain — fuse [posX, posZ, yawDeg, minLidarDist]
        // ImuSensor layout: [0]=pos.x [1]=pos.y [2]=pos.z
        //                   [3]=vel.x [4]=vel.y [5]=vel.z
        //                   [6]=rot.x [7]=rot.y(yaw) [8]=rot.z
        // LiDAR (180 rays) starts at index 9.
        .withPerceptionChain<4>([](const auto& rawObs) {
            std::array<float, 4> fused{};
            fused[0] = rawObs[0]; // IMU pos.x
            fused[1] = rawObs[2]; // IMU pos.z
            fused[2] = rawObs[7]; // IMU rot.y = yaw (degrees)

            float minLidar = 20.0f;
            for (std::size_t i = 9; i < 9 + 180; ++i) {
                minLidar = std::min(minLidar, rawObs[i]);
            }
            fused[3] = minLidar;
            return fused;
        })
        // Pillar 5: Actuators (Differential Mobile Drive: v, w)
        .withActuators(
            actuators::DifferentialDriveActuator{}
        )
        // Pillar 6: Planning Policy
        .withPolicy(AmrGoalSeekingPolicy{4.0f, -2.0f});

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT 3D WEBGPU VISUAL ARENA SCENE & SPAWN AGENT
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing 3D WebGPU Visual Arena & Spawning Agent...\n";

    // Attach SimArena directly to WebGPU App in a single fluent pipeline
    app.attachArena(
        makeArena<KinematicPhysicsEngine>()
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
            )
    );

    std::cout << "  - 3D WebGPU Visual Arena Environment initialized successfully!\n";
    std::cout << "  - Interactive 3D Window: Orbit/Pan Camera with WASD & Mouse!\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN 3D INTERACTIVE WEBGPU RENDERING & SIMULATION LOOP
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Launching 3D Interactive WebGPU Graphics Window...\n";
    app.run(runtime);

    tracer.logSummary();
    runtime.shutdown();
    std::cout << "\n[SimArena] 3D Visual Physical Agent Simulation application shutdown complete.\n";
    return 0;
}
