#pragma once

#include "corium_sim/kinematics/SimJoint.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::kinematics {

/// @brief Forward Kinematics Solver for Articulated Multi-Link Robotic Bodies.
class JointKinematics {
public:
    JointKinematics() = default;
    ~JointKinematics() = default;

    /// @brief Update world-space poses of all child link entities based on joint positions and parent hierarchy.
    void updateKinematics(scene::SimScene& scene, float deltaTime = 0.0f) noexcept;

    /// @brief Inverse Kinematics (IK) Solver: Compute joint angles to reach a target 3D end-effector position.
    bool solveIK(
        scene::SimScene& scene,
        const std::string& endEffectorName,
        const math::Vec3& targetPosition,
        uint32_t maxIterations = 50,
        float tolerance = 0.01f
    ) noexcept;
};

} // namespace corium_sim::kinematics
