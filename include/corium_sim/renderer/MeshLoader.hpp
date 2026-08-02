#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <algorithm>
#include <cmath>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Mesh.hpp"

namespace corium_sim::renderer {

/// @brief Axis-Aligned Bounding Box (AABB) for 3D Simulation Assets & Collision Bounds.
struct BoundingBox {
    math::Vec3 min{ 1e30f,  1e30f,  1e30f};
    math::Vec3 max{-1e30f, -1e30f, -1e30f};

    [[nodiscard]] math::Vec3 center() const noexcept
    {
        return math::Vec3{
            (min.x + max.x) * 0.5f,
            (min.y + max.y) * 0.5f,
            (min.z + max.z) * 0.5f
        };
    }

    [[nodiscard]] math::Vec3 extents() const noexcept
    {
        return math::Vec3{
            std::abs(max.x - min.x),
            std::abs(max.y - min.y),
            std::abs(max.z - min.z)
        };
    }

    [[nodiscard]] float maxExtent() const noexcept
    {
        math::Vec3 ext = extents();
        return std::max({ext.x, ext.y, ext.z});
    }

    void expand(const math::Vec3& p) noexcept
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);

        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
};

/// @brief Parsed 3D Mesh Geometry Data container.
struct MeshData {
    std::vector<Vertex> vertices{};
    std::vector<uint32_t> indices{};
    BoundingBox bounds{};
    std::string name{};
    bool success = false;
};

/// @brief High-performance Wavefront OBJ & 3D Simulation Asset Loader.
class MeshLoader {
public:
    /// @brief Parse Wavefront OBJ 3D mesh model file.
    static MeshData parseOBJFile(const std::string& filePath);

    /// @brief Parse Wavefront OBJ 3D model from in-memory string.
    static MeshData parseOBJMemory(std::string_view content);

private:
    static void generateNormalsIfMissing(MeshData& data);
};

} // namespace corium_sim::renderer
