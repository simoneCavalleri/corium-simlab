#pragma once

#include <cstdint>
#include <vector>
#include "corium_sim/math/Math.hpp"

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

namespace corium_sim::scene { class SimScene; }

namespace corium_sim::renderer {

/// @brief WebGPU WGSL Compute Shader Pipeline for parallel GPU sensor calculations (LiDAR Raycasting).
class WebGpuComputePipeline {
public:
    WebGpuComputePipeline() = default;
    ~WebGpuComputePipeline();

    WebGpuComputePipeline(const WebGpuComputePipeline&) = delete;
    WebGpuComputePipeline& operator=(const WebGpuComputePipeline&) = delete;

    WebGpuComputePipeline(WebGpuComputePipeline&& rhs) noexcept;
    WebGpuComputePipeline& operator=(WebGpuComputePipeline&& rhs) noexcept;

    bool initialize(WGPUDevice device) noexcept;
    void shutdown() noexcept;

    /// @brief Execute WGSL Compute Shader parallel 3D Raycasting against scene AABBs.
    bool computeLidarRaycast(
        WGPUDevice device,
        WGPUQueue queue,
        const std::vector<math::Vec3>& rayOrigins,
        const std::vector<math::Vec3>& rayDirections,
        const scene::SimScene& scene,
        float maxDistance,
        std::vector<float>& outDistances
    ) noexcept;

private:
    void moveFrom(WebGpuComputePipeline&& rhs) noexcept;

    WGPUBindGroupLayout _computeBindGroupLayout = nullptr;
    WGPUComputePipeline _computePipeline = nullptr;
    bool _initialized = false;
};

} // namespace corium_sim::renderer
