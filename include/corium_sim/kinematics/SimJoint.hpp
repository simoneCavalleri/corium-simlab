#pragma once

#include <cstdint>
#include <string>
#include "corium_sim/math/Math.hpp"

namespace corium_sim::kinematics {

/// @brief Type of kinematic joint connecting parent and child robot links.
enum class JointType : uint8_t {
    Revolute,  // Rotational joint around axis (radians)
    Prismatic, // Translational linear joint along axis (meters)
    Fixed      // Rigid 0-DOF connection
};

/// @brief Articulated Joint connecting parent and child SimEntity links.
struct SimJoint {
    uint32_t id = 0;
    std::string name{};
    std::string parentName{};
    std::string childName{};

    JointType type = JointType::Revolute;
    math::Vec3 anchor{0.0f, 0.0f, 0.0f}; // Anchor location relative to parent entity origin
    math::Vec3 axis{0.0f, 1.0f, 0.0f};   // Unit axis of rotation or translation

    float position = 0.0f;       // Current joint coordinate (radians for Revolute, meters for Prismatic)
    float minLimit = -3.14159f;  // Minimum joint limit
    float maxLimit =  3.14159f;  // Maximum joint limit
    float targetVelocity = 0.0f; // Actuator velocity target

    // Fast O(1) Direct Vector Index Caches (resolved on first frame)
    mutable std::size_t parentEntityIndex = static_cast<std::size_t>(-1);
    mutable std::size_t childEntityIndex = static_cast<std::size_t>(-1);
};

} // namespace corium_sim::kinematics
