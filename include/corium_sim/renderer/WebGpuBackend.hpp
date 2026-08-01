#pragma once

#include <cstdint>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Camera.hpp"
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

struct GLFWwindow;

namespace corium_sim::renderer {

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

    /// @brief Set background clear color for render pass.
    void setClearColor(double r, double g, double b, double a = 1.0) noexcept;

    /// @brief Resize WebGPU surface swapchain and depth buffer.
    void resize(uint32_t width, uint32_t height) noexcept;

    /// @brief Begin 3D Render Pass frame with Camera setup.
    bool beginFrame(const Camera& camera, float time = 0.0f) noexcept;

    /// @brief Render 3D Mesh with specified Texture and Model Transformation Matrix.
    void drawMesh(const Mesh& mesh, const Texture& texture, const math::Mat4& modelMatrix) noexcept;

    /// @brief End Render Pass, submit command buffer, and present surface.
    void endFrame() noexcept;

    /// @brief Legacy simple render pass method.
    void renderFrame(double deltaTime) noexcept;

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
    void createSurface();
    void configureSurface();
    void createDepthBuffer();
    void createPipelineLayout();
    void createRenderPipeline();
    void moveFrom(WebGpuBackend&& rhs) noexcept;

    WGPUBindGroup createBindGroup(const Texture& texture) noexcept;

    WGPUInstance _instance = nullptr;
    WGPUAdapter _adapter = nullptr;
    WGPUDevice _device = nullptr;
    WGPUQueue _queue = nullptr;
    WGPUSurface _surface = nullptr;

    WGPUTexture _depthTexture = nullptr;
    WGPUTextureView _depthTextureView = nullptr;

    WGPUBindGroupLayout _bindGroupLayout = nullptr;
    WGPUPipelineLayout _pipelineLayout = nullptr;
    WGPURenderPipeline _pipeline = nullptr;
    WGPUBuffer _uniformBuffer = nullptr;

    WGPURenderPassEncoder _currentPass = nullptr;
    WGPUCommandEncoder _currentEncoder = nullptr;
    WGPUTexture _currentSurfaceTexture = nullptr;
    WGPUTextureView _currentColorView = nullptr;
    std::vector<WGPUBindGroup> _activeBindGroups{};

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
