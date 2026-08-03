#pragma once

#include <array>
#include <cstddef>
#include <span>
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/scene/SimEntity.hpp"

namespace corium_sim::agent::actuators {

/// @brief Target Joint Position Actuator (action_size = JointCount).
template <std::size_t JointCount>
class JointPositionActuator {
public:
    static constexpr std::size_t action_size = JointCount;

    void apply([[maybe_unused]] scene::SimEntity& entity, std::span<const float> action) noexcept
    {
        std::size_t count = std::min(JointCount, action.size());
        for (std::size_t i = 0; i < count; ++i) {
            _targetPositions[i] = action[i];
        }
    }

    [[nodiscard]] std::span<const float> targetPositions() const noexcept
    {
        return std::span<const float>(_targetPositions.data(), JointCount);
    }

private:
    std::array<float, JointCount> _targetPositions{};
};

/// @brief Target Joint Velocity Actuator (action_size = JointCount).
template <std::size_t JointCount>
class JointVelocityActuator {
public:
    static constexpr std::size_t action_size = JointCount;

    void apply([[maybe_unused]] scene::SimEntity& entity, std::span<const float> action) noexcept
    {
        std::size_t count = std::min(JointCount, action.size());
        for (std::size_t i = 0; i < count; ++i) {
            _targetVelocities[i] = action[i];
        }
    }

    [[nodiscard]] std::span<const float> targetVelocities() const noexcept
    {
        return std::span<const float>(_targetVelocities.data(), JointCount);
    }

private:
    std::array<float, JointCount> _targetVelocities{};
};

/// @brief Differential Mobile Drive Actuator (2 floats: linear velocity v, angular velocity w).
class DifferentialDriveActuator {
public:
    static constexpr std::size_t action_size = 2;

    void apply(scene::SimEntity& entity, std::span<const float> action) noexcept
    {
        if (action.size() < 2) return;
        float linearVel = action[0];
        float angularVel = action[1]; // deg/sec

        // Update linear and angular velocities of physical body
        entity.velocity.z = linearVel;
        entity.angularVelocity.y = angularVel;
    }
};

} // namespace corium_sim::agent::actuators
