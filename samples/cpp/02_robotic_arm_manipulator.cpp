// =============================================================================
// Corium SimLab Sample #02 — Industrial 3-DOF Robot Arm Pick-and-Place Manipulation
// =============================================================================

#include <iostream>
#include <iomanip>
#include <cmath>
#include "corium_sim/SimLab.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"

using namespace corium_sim;
using namespace corium_sim::agent;
using namespace corium_sim::environment;
using namespace corium_sim::physics;
using namespace corium_sim::math;
using namespace corium_sim::renderer;
using namespace corium_sim::scene;
using namespace corium_sim::kinematics;

// =============================================================================
// Industrial Robot Arm Joint Manipulation Policy (PolicyConcept)
// =============================================================================
class RobotArmJointControlPolicy {
public:
    /// @brief Plan target positions for 3 articulated robotic arm joints (Base, Shoulder, Elbow).
    template <typename ObservationBuffer>
    [[nodiscard]] inline std::array<float, 3> plan(const ObservationBuffer& jointObs) noexcept
    {
        _simTime += 0.01667f;
        std::array<float, 3> action{};

        // Smooth sinusoidal trajectory for 3-DOF pick-and-place operation
        action[0] = std::sin(_simTime * 1.2f) * 0.9f; // Joint Base Yaw rotation
        action[1] = std::cos(_simTime * 1.8f) * 0.4f; // Joint Shoulder Pitch
        action[2] = std::sin(_simTime * 2.2f) * 0.5f; // Joint Elbow Pitch

        (void)jointObs;
        return action;
    }

private:
    float _simTime = 0.0f;
};

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — Sample #02: Industrial 3-DOF Robot Arm Manipulation\n";
    std::cout << " Features: 3D WebGPU Graphics | URDF Robot Loading | Joint Position Control\n";
    std::cout << "=========================================================================\n\n";

    SimRuntime runtime;
    SimLabApp app;

    runtime.initialize(app);

    // -------------------------------------------------------------------------
    // 1. DEFINE REUSABLE AGENT SPECIFICATION BLUEPRINT FOR ROBOTIC MANIPULATOR
    // -------------------------------------------------------------------------
    std::cout << "[Step 1] Constructing Decoupled Robotic Arm AgentSpec Blueprint...\n";

    auto robotArmSpec = makeAgentSpec()
        // Pillar 2: Agent Model
        .withModel(SimEntity{
            .name = "base_link",
            .position = {0.0f, 0.4f, 0.0f},
            .isStatic = true
        })
        // Pillar 3: Onboard Joint Encoder Sensors
        .withSensors(
            sensors::JointEncoderSensor<3>{}
        )
        // Pillar 4: Perception Chain
        .withoutPerceptionChain()
        // Pillar 5: Actuators (3-DOF Joint Position Actuators for URDF Arm)
        .withActuators(
            actuators::JointPositionActuator<3>{}
        )
        // Pillar 6: Planning Policy (Trajectory & Pick-and-Place Joint Controller)
        .withPolicy(RobotArmJointControlPolicy{});

    // -------------------------------------------------------------------------
    // 2. CONSTRUCT INDUSTRIAL WORKSTATION SCENE & SPAWN ROBOT ARM AGENT
    // -------------------------------------------------------------------------
    std::cout << "[Step 2] Constructing Industrial Workstation Scene & Spawning Robot Arm...\n";

    auto arena = makeArena<KinematicPhysicsEngine>()
        // Pillar 1: Environment & 3D Scene Definition
        .withEnvironment([](SimScene& scene) {
            // Factory Floor Grid
            scene.addEntity(SimEntity{.name = "user_ground", .shape = EntityShape::PlaneGrid, .isStatic = true});

            // Heavy Industrial Pedestal Table Workstation
            scene.addEntity(SimEntity{
                .name = "workstation_table",
                .material = Material::Metallic({0.4f, 0.45f, 0.50f, 1.0f}, 0.35f),
                .position = {0.0f, 0.4f, 0.0f},
                .scale = {3.0f, 0.8f, 2.0f},
                .isStatic = true
            });

            // Load URDF Articulated Robot Arm & Joints
            UrdfLoader::loadURDF("assets/urdf/sample_arm.urdf", scene, nullptr, nullptr, Vec3{0.0f, 0.4f, 0.0f});


            // Parallel Two-Finger Gripper End-Effector
            scene.addEntity(SimEntity{
                .name = "gripper_finger_l",
                .material = Material::Matte({0.2f, 0.2f, 0.25f, 1.0f}),
                .position = {-0.15f, 3.2f, 0.0f},
                .scale = {0.08f, 0.3f, 0.12f}
            });
            scene.addEntity(SimEntity{
                .name = "gripper_finger_r",
                .material = Material::Matte({0.2f, 0.2f, 0.25f, 1.0f}),
                .position = {0.15f, 3.2f, 0.0f},
                .scale = {0.08f, 0.3f, 0.12f}
            });

            // Workpiece Inspection Platform & Red Metallic Target Workpiece
            scene.addEntity(SimEntity{
                .name = "inspection_platform",
                .material = Material::Metallic({0.5f, 0.52f, 0.55f, 1.0f}, 0.40f),
                .position = {3.5f, 0.5f, -1.5f},
                .scale = {2.0f, 1.0f, 1.5f},
                .isStatic = true
            });
            scene.addEntity(SimEntity{
                .name = "target_workpiece",
                .material = Material::Metallic({0.95f, 0.10f, 0.15f, 1.0f}, 0.10f),
                .position = {3.5f, 1.25f, -1.5f},
                .scale = {0.6f, 0.5f, 0.6f}
            });
        })
        .spawnAgent("robot_arm", std::move(robotArmSpec), Vec3{0.0f, 0.4f, 0.0f})
        // Pillar 7: Reward Policy
        .withRewardPolicy(
            RewardBuilder{}
                .addTerm<DistanceToGoalPenalty>(1.0f)
                .addTerm<GoalReachedBonus>(100.0f)
        );

    auto arenaPtr = std::make_shared<decltype(arena)>(std::move(arena));

    // Connect shared 3D scene & arena to WebGPU App
    app.attachArena(arenaPtr);

    // Register additional joint target synchronization callback
    app.onStep([arenaPtr](SimLabApp& app, float dt) {
        (void)arenaPtr->step(dt);

        auto jointTargets = arenaPtr->agent().actuators().template getActuator<0>().targetPositions();
        if (jointTargets.size() >= 3) {
            if (auto* jBase     = app.scene().findJoint("joint_base"))     jBase->position     = jointTargets[0];
            if (auto* jShoulder = app.scene().findJoint("joint_shoulder")) jShoulder->position = jointTargets[1];
            if (auto* jElbow    = app.scene().findJoint("joint_elbow"))    jElbow->position    = jointTargets[2];
        }
    });

    std::cout << "  - 3D Industrial Workstation & URDF Robot Arm Initialized Successfully!\n\n";

    // -------------------------------------------------------------------------
    // 3. RUN 3D INTERACTIVE WEBGPU RENDERING & SIMULATION LOOP
    // -------------------------------------------------------------------------
    std::cout << "[Step 3] Launching 3D Interactive WebGPU Graphics Window...\n";
    app.run(runtime);

    runtime.shutdown();
    std::cout << "\n[SimArena] Industrial Robot Arm application shutdown complete.\n";
    return 0;
}
