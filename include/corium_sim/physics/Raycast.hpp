#pragma once

#include <string>
#include <vector>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::physics {

/// @brief Struct representing a 3D Raycast Hit result.
struct RaycastHit {
    bool hit = false;
    float distance = 0.0f;
    math::Vec3 point{0.0f, 0.0f, 0.0f};
    math::Vec3 normal{0.0f, 1.0f, 0.0f};
    std::string entityName;
};

/// @brief 3D Physics Raycasting & LiDAR Proximity Sensor Engine.
class Raycast {
public:
    /// @brief Cast a single 3D ray into the scene and return closest entity intersection hit.
    static RaycastHit castRay(
        const scene::SimScene& scene,
        const math::Vec3& origin,
        const math::Vec3& direction,
        float maxDistance = 50.0f
    ) noexcept;

    /// @brief Perform a 360-degree horizontal 3D LiDAR point cloud scan around sensor origin.
    static std::vector<RaycastHit> castLidarScan(
        const scene::SimScene& scene,
        const math::Vec3& origin,
        const math::Vec3& forwardDir = {0.0f, 0.0f, 1.0f},
        uint32_t numRays = 36,
        float fovDegrees = 360.0f,
        float maxDistance = 20.0f
    ) noexcept;
};

} // namespace corium_sim::physics
