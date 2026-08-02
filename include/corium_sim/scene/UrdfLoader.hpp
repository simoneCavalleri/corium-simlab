#pragma once

#include <string>
#include <vector>
#include "corium_sim/scene/SimScene.hpp"

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

namespace corium_sim::scene {

/// @brief URDF (Unified Robot Description Format) XML Parser & Robot Scene Construction Utility.
class UrdfLoader {
public:
    /// @brief Parse URDF XML specification file and append entities/joints to target SimScene.
    static bool loadURDF(
        const std::string& urdfFilePath,
        SimScene& scene,
        WGPUDevice device = nullptr,
        WGPUQueue queue = nullptr,
        const math::Vec3& basePosition = {0.0f, 0.0f, 0.0f},
        const math::Vec3& baseScale = {1.0f, 1.0f, 1.0f}
    );
};

} // namespace corium_sim::scene
