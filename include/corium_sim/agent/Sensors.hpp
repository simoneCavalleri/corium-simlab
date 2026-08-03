#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/math/Math.hpp"
#include "corium_sim/physics/Raycast.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent::sensors {

/// @brief 3D Position & Linear Velocity Sensor (6 floats: pos_x, pos_y, pos_z, vel_x, vel_y, vel_z).
class PositionEncoderSensor {
public:
    static constexpr std::size_t observation_size = 6;

    [[nodiscard]] std::span<const float> sample(const scene::SimEntity& entity, [[maybe_unused]] const scene::SimScene& scene) noexcept
    {
        _buffer[0] = entity.position.x;
        _buffer[1] = entity.position.y;
        _buffer[2] = entity.position.z;
        _buffer[3] = entity.velocity.x;
        _buffer[4] = entity.velocity.y;
        _buffer[5] = entity.velocity.z;
        return std::span<const float>(_buffer.data(), observation_size);
    }

private:
    std::array<float, observation_size> _buffer{};
};

/// @brief 6-DOF Inertial Measurement Unit (IMU) Sensor (9 floats: pos, vel, rot_deg).
class ImuSensor {
public:
    static constexpr std::size_t observation_size = 9;

    [[nodiscard]] std::span<const float> sample(const scene::SimEntity& entity, [[maybe_unused]] const scene::SimScene& scene) noexcept
    {
        _buffer[0] = entity.position.x;
        _buffer[1] = entity.position.y;
        _buffer[2] = entity.position.z;
        _buffer[3] = entity.velocity.x;
        _buffer[4] = entity.velocity.y;
        _buffer[5] = entity.velocity.z;
        _buffer[6] = entity.rotation.x;
        _buffer[7] = entity.rotation.y;
        _buffer[8] = entity.rotation.z;
        return std::span<const float>(_buffer.data(), observation_size);
    }

private:
    std::array<float, observation_size> _buffer{};
};

/// @brief Joint Position & Velocity Encoder Sensor (JointCount * 2 floats).
template <std::size_t JointCount>
class JointEncoderSensor {
public:
    static constexpr std::size_t observation_size = JointCount * 2;

    constexpr JointEncoderSensor() = default;

    /// @brief Update internal simulated joint state measurements.
    void updateJoints(std::span<const float> angles, std::span<const float> velocities) noexcept
    {
        for (std::size_t i = 0; i < JointCount; ++i) {
            _angles[i] = (i < angles.size()) ? angles[i] : 0.0f;
            _velocities[i] = (i < velocities.size()) ? velocities[i] : 0.0f;
        }
    }

    [[nodiscard]] std::span<const float> sample([[maybe_unused]] const scene::SimEntity& entity, [[maybe_unused]] const scene::SimScene& scene) noexcept
    {
        for (std::size_t i = 0; i < JointCount; ++i) {
            _buffer[i] = _angles[i];
            _buffer[JointCount + i] = _velocities[i];
        }
        return std::span<const float>(_buffer.data(), observation_size);
    }

private:
    std::array<float, JointCount> _angles{};
    std::array<float, JointCount> _velocities{};
    std::array<float, observation_size> _buffer{};
};

/// @brief 3D Raycasting LiDAR Sensor performing real-time raycasts against 3D environment scene.
template <std::size_t RayCount>
class RaycastLidarSensor {
public:
    static constexpr std::size_t observation_size = RayCount;

    float maxDistance = 20.0f;
    float fovDegrees = 360.0f;

    [[nodiscard]] std::span<const float> sample(const scene::SimEntity& entity, const scene::SimScene& scene) noexcept
    {
        math::Vec3 origin = entity.position + math::Vec3{0.0f, 0.5f, 0.0f};
        float startAngle = -fovDegrees * 0.5f * math::DEG2RAD;
        float angleStep = (RayCount > 1) ? (fovDegrees * math::DEG2RAD / static_cast<float>(RayCount)) : 0.0f;
        float yawRad = entity.rotation.y * math::DEG2RAD;

        for (std::size_t i = 0; i < RayCount; ++i) {
            float rayAngle = yawRad + startAngle + static_cast<float>(i) * angleStep;
            math::Vec3 dir{std::sin(rayAngle), 0.0f, std::cos(rayAngle)};

            auto hit = physics::Raycast::castRay(scene, origin, dir, maxDistance);
            _distances[i] = hit.hit ? hit.distance : maxDistance;
        }
        return std::span<const float>(_distances.data(), RayCount);
    }

private:
    std::array<float, RayCount> _distances{};
};

} // namespace corium_sim::agent::sensors
