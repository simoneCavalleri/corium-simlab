#pragma once

#include "corium_sim/math/Math.hpp"

namespace corium_sim::renderer {

/// @brief Physically Based Rendering (PBR) Material Properties.
struct Material {
    math::Vec4 albedo{0.8f, 0.8f, 0.8f, 1.0f}; // RGBA Albedo Color Tint
    float metallic = 0.1f;                       // 0.0 = Dielectric, 1.0 = Pure Metal
    float roughness = 0.5f;                      // 0.0 = Smooth / Glossy Mirror, 1.0 = Rough Matte
    float emissive = 0.0f;                       // Emissive light intensity

    /// @brief Pre-defined Material Presets
    static Material Metallic(const math::Vec4& color = {0.9f, 0.91f, 0.92f, 1.0f}, float roughness = 0.25f) noexcept
    {
        return Material{ .albedo = color, .metallic = 0.95f, .roughness = roughness };
    }

    static Material Matte(const math::Vec4& color) noexcept
    {
        return Material{ .albedo = color, .metallic = 0.0f, .roughness = 0.9f };
    }

    static Material Glossy(const math::Vec4& color) noexcept
    {
        return Material{ .albedo = color, .metallic = 0.1f, .roughness = 0.15f };
    }
};

} // namespace corium_sim::renderer
