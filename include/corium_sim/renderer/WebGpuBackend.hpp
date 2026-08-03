#pragma once

#include <cstdint>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/Texture.hpp"
#include "corium_sim/renderer/Uniforms.hpp"

#include <vector>

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

namespace corium_sim::scene { class SimScene; }
struct GLFWwindow;

namespace corium_sim::renderer {

/// @brief WebGPU Offscreen Render Target for Agent Visual Sensors (RGB & Depth).
struct OffscreenTarget {
    WGPUTexture colorTexture = nullptr;
    WGPUTextureView colorView = nullptr;
    WGPUTexture depthTexture = nullptr;
    WGPUTextureView depthView = nullptr;
    WGPUBuffer stagingBuffer = nullptr;
    uint32_t width = 128;
    uint32_t height = 128;
    uint32_t bytesPerRow = 0;
    uint32_t bufferSize = 0;
    bool isValid = false;
};

/// @brief WebGPU 3D Rendering Engine Backend.
/// Manages WebGPU Swapchain, Depth Buffer, WGSL Shaders, Pipeline Layout, BindGroups, and Render Pass.
class WebGpuBackend {
public:
    WebGpuBackend();
    ~WebGpuBackend();

    WebGpuBackend(const WebGpuBackend&) = delete;
    WebGpuBackend& operator=(const WebGpuBackend&) = delete;

    WebGpuBackend(WebGpuBackend&& rhs) noexcept;
    WebGpuBackend& operator=(WebGpuBackend&& rhs) noexcept;

    /// @brief Initialize WebGPU context, Depth Buffer, Pipeline, Default Textures, and WGSL Shaders.
    bool initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height);

    /// @brief Create an Offscreen Render Target for agent visual cameras.
    OffscreenTarget createOffscreenTarget(uint32_t width = 128, uint32_t height = 128) noexcept;

    /// @brief Render 3D Scene into an Offscreen Render Target from Camera perspective.
    void renderOffscreen(const OffscreenTarget& target, const Camera& camera, const scene::SimScene& scene) noexcept;

    /// @brief Copy offscreen color texture to CPU staging buffer.
    bool copyOffscreenToStaging(const OffscreenTarget& target) noexcept;

    /// @brief Read raw RGBA8 pixel payload from offscreen render target to CPU buffer.
    std::vector<uint8_t> readOffscreenPixels(const OffscreenTarget& target) noexcept;

    /// @brief Release and destroy an Offscreen Render Target.
    void destroyOffscreenTarget(OffscreenTarget& target) noexcept;

    /// @brief Set background clear color for render pass.
    void setClearColor(double r, double g, double b, double a = 1.0) noexcept;

    /// @brief Resize WebGPU surface swapchain and depth buffer.
    void resize(uint32_t width, uint32_t height) noexcept;

    /// @brief Begin 3D Render Pass frame with Camera setup.
    bool beginFrame(const Camera& camera, float time = 0.0f) noexcept;

    /// @brief Render 3D Mesh with specified Texture, Model Matrix, and PBR Material.
    void drawMesh(
        const Mesh& mesh,
        const Texture& texture,
        const math::Mat4& modelMatrix,
        const Material& material = {}
    ) noexcept;

    /// @brief End Render Pass, submit command buffer, and present surface.
    void endFrame() noexcept;

    /// @brief Shutdown WebGPU resources cleanly.
    void shutdown() noexcept;

    [[nodiscard]] bool isInitialized() const noexcept { return _initialized; }
    [[nodiscard]] uint64_t frameCount() const noexcept { return _frameCount; }
    [[nodiscard]] uint32_t width() const noexcept { return _width; }
    [[nodiscard]] uint32_t height() const noexcept { return _height; }
    [[nodiscard]] WGPUDevice device() const noexcept { return _device; }
    [[nodiscard]] WGPUQueue queue() const noexcept { return _queue; }

    [[nodiscard]] const Texture& defaultCheckerTexture() const noexcept { return _defaultCheckerTex; }
    [[nodiscard]] const Texture& defaultGridTexture() const noexcept { return _defaultGridTex; }

private:
    struct BindGroupCacheEntry {
        WGPUBindGroup bindGroup = nullptr;
        WGPUTextureView textureView = nullptr;
        WGPUSampler sampler = nullptr;
        WGPUBuffer uboBuffer = nullptr;
    };

    void createSurface();
    void configureSurface();
    void createDepthBuffer();
    void createPipelineLayout();
    void createRenderPipeline();
    void clearRenderPools() noexcept;
    void moveFrom(WebGpuBackend&& rhs) noexcept;

    WGPUBindGroup getOrCreateBindGroup(size_t index, const Texture& texture, WGPUBuffer uboBuffer) noexcept;
    WGPUBuffer getOrCreateUboBuffer(size_t index) noexcept;

    WGPUInstance _instance = nullptr;
    WGPUAdapter _adapter = nullptr;
    WGPUDevice _device = nullptr;
    WGPUQueue _queue = nullptr;
    WGPUSurface _surface = nullptr;
    WGPUTextureFormat _surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

    WGPUTexture _depthTexture = nullptr;
    WGPUTextureView _depthTextureView = nullptr;

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

    Texture _defaultCheckerTex{};
    Texture _defaultGridTex{};

    UniformBufferObject _currentUbo{};
    GLFWwindow* _window = nullptr;

    uint32_t _width = 1280;
    uint32_t _height = 720;
    uint64_t _frameCount = 0;
    WGPUColor _clearColor{ 0.08, 0.09, 0.12, 1.0 };
    bool _initialized = false;
    bool _inFrame = false;
};

} // namespace corium_sim::renderer
