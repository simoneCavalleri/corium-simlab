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
};

} // namespace corium_sim::kinematics
