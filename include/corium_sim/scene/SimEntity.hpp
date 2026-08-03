#pragma once

#include <cstdint>
#include <string>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/MeshLoader.hpp"
#include "corium_sim/renderer/Texture.hpp"

namespace corium_sim::scene {

enum class EntityShape { Cube, PlaneGrid, Sphere, Cylinder, MeshModel };

/// @brief 3D Entity in the Simulation Scene (Robots, Obstacles, Targets, Terrains).
struct SimEntity {
    uint32_t id = 0;
    std::string name{};
    EntityShape shape = EntityShape::Cube;

    renderer::Mesh mesh{};
    renderer::Texture texture{};
    renderer::Material material{};


    math::Vec3 position{0.0f, 0.0f, 0.0f};
    math::Vec3 rotation{0.0f, 0.0f, 0.0f}; // Euler angles in degrees (X, Y, Z)
    math::Vec3 scale{1.0f, 1.0f, 1.0f};
    math::Vec3 visualOffset{0.0f, 0.0f, 0.0f}; // Offset from joint/pivot origin to mesh center

    // 3D Physics Properties
    math::Vec3 velocity{0.0f, 0.0f, 0.0f};
    math::Vec3 angularVelocity{0.0f, 0.0f, 0.0f}; // Degrees / sec
    float mass{1.0f};
    bool isStatic{false};
    bool hasPhysics{true};

    renderer::BoundingBox localBounds{};

    /// @brief Compute 4x4 Model Transformation Matrix (TRS) matching WebGPU WGSL shader.
    [[nodiscard]] math::Mat4 transformMatrix() const noexcept
    {
        math::Mat4 T = math::Mat4::translate(position);
        math::Mat4 Rx = math::Mat4::rotateX(rotation.x * math::DEG2RAD);
        math::Mat4 Ry = math::Mat4::rotateY(rotation.y * math::DEG2RAD);
        math::Mat4 Rz = math::Mat4::rotateZ(rotation.z * math::DEG2RAD);
        math::Mat4 R = Ry * Rx * Rz;
        math::Mat4 T_off = math::Mat4::translate(visualOffset);
        math::Mat4 S = math::Mat4::scale(scale);

        return T * R * T_off * S;
    }

    /// @brief Compute World-Space Axis-Aligned Bounding Box (AABB).
    [[nodiscard]] renderer::BoundingBox worldBounds() const noexcept
    {
        renderer::BoundingBox wb{};
        math::Mat4 model = transformMatrix();

        // Transform 8 corners of local AABB
        math::Vec3 minP = localBounds.min;
        math::Vec3 maxP = localBounds.max;

        math::Vec3 corners[8] = {
            {minP.x, minP.y, minP.z},
            {maxP.x, minP.y, minP.z},
            {minP.x, maxP.y, minP.z},
            {maxP.x, maxP.y, minP.z},
            {minP.x, minP.y, maxP.z},
            {maxP.x, minP.y, maxP.z},
            {minP.x, maxP.y, maxP.z},
            {maxP.x, maxP.y, maxP.z}
        };

        for (const auto& c : corners) {
            math::Vec4 worldP4 = model * math::Vec4{c.x, c.y, c.z, 1.0f};
            wb.expand(math::Vec3{worldP4.x, worldP4.y, worldP4.z});
        }

        return wb;
    }
};

} // namespace corium_sim::scene
