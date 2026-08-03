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
// User Planning Policy implementing PolicyConcept
// =============================================================================
class UserPlannerPolicy {
public:
    UserPlannerPolicy(float targetX, float targetZ)
        : _targetX(targetX), _targetZ(targetZ) {}

    template <typename ObservationBuffer>
    [[nodiscard]] inline std::array<float, 8> plan(const ObservationBuffer& fusedObs) noexcept
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

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — 3D WebGPU Interactive Visual Physical Agent Incubator\n";
    std::cout << " Features: 3D WebGPU Window | Real-Time LiDAR Raycasting | Visual Camera\n";
    std::cout << "=========================================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AGENT SPECIFICATION BLUEPRINT
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Decoupled AgentSpec Blueprint...\n";

    auto amrAgentSpec = makeAgentSpec()
        .withModel(SimEntity{.name = "amr_robot", .position = {0.0f, 0.5f, 0.0f}, .mass = 15.0f})
        .withSensors(
            sensors::ImuSensor{},
            sensors::JointEncoderSensor<6>{},
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
        .withActuators(
            actuators::JointPositionActuator<6>{},
            actuators::DifferentialDriveActuator{}
        )
        .withPolicy(UserPlannerPolicy{4.0f, -2.0f});

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT 3D WEBGPU VISUAL INCUBATOR SCENE & SPAWN AGENT
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing 3D WebGPU Visual Scene & Spawning Agent...\n";

    auto incubator = makeIncubator<KinematicPhysicsEngine>()
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
        .withRewardPolicy(
            RewardBuilder{}
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<CollisionPenalty>(50.0f)
                .addTerm<GoalReachedBonus>(200.0f)
        );

    auto incubatorPtr = std::make_shared<decltype(incubator)>(std::move(incubator));

    // Attach incubator step callback to generic SimLabApp
    app.onStep([incubatorPtr](SimLabApp&, float dt) {
        incubatorPtr->step(dt);
    });

    app.setScene(std::move(incubatorPtr->env().scene()));


    std::cout << "  - 3D WebGPU Visual Environment set successfully!\n";
    std::cout << "  - Interactive 3D Window: Orbit/Pan Camera with WASD & Mouse!\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN 3D INTERACTIVE WEBGPU RENDERING & SIMULATION LOOP (60 FPS)
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Launching 3D Interactive WebGPU Graphics Window...\n";
    app.run(runtime);

    runtime.shutdown();
    std::cout << "\n[Incubator] 3D Visual Physical Agent Incubator application shutdown complete.\n";
    return 0;
}
