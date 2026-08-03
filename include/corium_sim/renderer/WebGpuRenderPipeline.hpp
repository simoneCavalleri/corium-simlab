#pragma once

#include <cstdint>
#include <vector>
#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/Texture.hpp"
#include "corium_sim/renderer/Uniforms.hpp"

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

namespace corium_sim::renderer {

/// @brief WebGPU 3D Forward Mesh Render Pass, Shaders, PBR Uniforms, and BindGroup Cache Manager.
class WebGpuRenderPipeline {
public:
    WebGpuRenderPipeline() = default;
    ~WebGpuRenderPipeline();

    WebGpuRenderPipeline(const WebGpuRenderPipeline&) = delete;
    WebGpuRenderPipeline& operator=(const WebGpuRenderPipeline&) = delete;

    WebGpuRenderPipeline(WebGpuRenderPipeline&& rhs) noexcept;
    WebGpuRenderPipeline& operator=(WebGpuRenderPipeline&& rhs) noexcept;

    bool initialize(WGPUDevice device, WGPUQueue queue, WGPUTextureFormat surfaceFormat) noexcept;
    void shutdown() noexcept;

    bool beginFrame(
        WGPUDevice device,
        WGPUQueue queue,
        WGPUSurface surface,
        WGPUTextureView depthTextureView,
        const Camera& camera,
        WGPUColor clearColor,
        float time = 0.0f
    ) noexcept;

    bool beginOffscreenFrame(
        WGPUDevice device,
        WGPUQueue queue,
        WGPUTextureView colorTextureView,
        WGPUTextureView depthTextureView,
        const Camera& camera,
        WGPUColor clearColor,
        float time = 0.0f
    ) noexcept;

    void drawMesh(
        WGPUQueue queue,
        const Mesh& mesh,
        const Texture& texture,
        const math::Mat4& modelMatrix,
        const Material& material = {}
    ) noexcept;

    void endFrame(WGPUQueue queue, WGPUSurface surface = nullptr) noexcept;
    void endOffscreenFrame(WGPUQueue queue) noexcept;

    [[nodiscard]] bool inFrame() const noexcept { return _inFrame; }
    [[nodiscard]] const Texture& defaultWhiteTexture() const noexcept { return _defaultWhiteTex; }
    [[nodiscard]] const Texture& defaultCheckerTexture() const noexcept { return _defaultCheckerTex; }
    [[nodiscard]] const Texture& defaultGridTexture() const noexcept { return _defaultGridTex; }

private:
    struct BindGroupCacheEntry {
        WGPUBindGroup bindGroup = nullptr;
        WGPUTextureView textureView = nullptr;
        WGPUSampler sampler = nullptr;
        WGPUBuffer uboBuffer = nullptr;
    };

    void createPipelineLayout(WGPUDevice device) noexcept;
    void createRenderPipeline(WGPUDevice device, WGPUTextureFormat surfaceFormat) noexcept;
    void clearRenderPools() noexcept;
    WGPUBindGroup getOrCreateBindGroup(WGPUDevice device, size_t index, const Texture& texture, WGPUBuffer uboBuffer) noexcept;
    WGPUBuffer getOrCreateUboBuffer(WGPUDevice device, size_t index) noexcept;
    void moveFrom(WebGpuRenderPipeline&& rhs) noexcept;

    WGPUBindGroupLayout _bindGroupLayout = nullptr;
    WGPUPipelineLayout _pipelineLayout = nullptr;
    WGPURenderPipeline _pipeline = nullptr;
    WGPUBuffer _uniformBuffer = nullptr;

    std::vector<WGPUBuffer> _uboPool{};
    std::vector<BindGroupCacheEntry> _bindGroupPool{};
    size_t _currentUboIndex = 0;

    WGPURenderPassEncoder _currentPass = nullptr;
    WGPUCommandEncoder _currentEncoder = nullptr;
    WGPUTexture _currentSurfaceTexture = nullptr;
    WGPUTextureView _currentColorView = nullptr;

    Texture _defaultWhiteTex{};
    Texture _defaultCheckerTex{};
    Texture _defaultGridTex{};
    UniformBufferObject _currentUbo{};
    WGPUDevice _device = nullptr;
    bool _initialized = false;
    bool _inFrame = false;
};

} // namespace corium_sim::renderer
