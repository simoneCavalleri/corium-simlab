#pragma once

#include <cstdint>
#include <vector>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/Texture.hpp"
#include "corium_sim/renderer/WebGpuContext.hpp"
#include "corium_sim/renderer/WebGpuRenderPipeline.hpp"
#include "corium_sim/renderer/WebGpuComputePipeline.hpp"

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

/// @brief High-level WebGPU Renderer Facade coordinating Context, 3D Render Pipeline, and Compute Shader Pipeline.
class WebGpuBackend {
public:
    WebGpuBackend() = default;
    ~WebGpuBackend();

    WebGpuBackend(const WebGpuBackend&) = delete;
    WebGpuBackend& operator=(const WebGpuBackend&) = delete;

    WebGpuBackend(WebGpuBackend&& rhs) noexcept;
    WebGpuBackend& operator=(WebGpuBackend&& rhs) noexcept;

    /// @brief Initialize WebGPU context, Render Pipeline, Compute Pipeline, and Default Textures.
    bool initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height) noexcept;

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

    /// @brief Execute WebGPU WGSL Compute Shader parallel 3D Raycasting against scene AABBs.
    bool computeLidarRaycast(
        const std::vector<math::Vec3>& rayOrigins,
        const std::vector<math::Vec3>& rayDirections,
        const scene::SimScene& scene,
        float maxDistance,
        std::vector<float>& outDistances
    ) noexcept;

    [[nodiscard]] bool isInitialized() const noexcept { return _context.isInitialized(); }
    [[nodiscard]] uint64_t frameCount() const noexcept { return _frameCount; }
    [[nodiscard]] uint32_t width() const noexcept { return _context.width(); }
    [[nodiscard]] uint32_t height() const noexcept { return _context.height(); }

    [[nodiscard]] WGPUDevice device() const noexcept { return _context.device(); }
    [[nodiscard]] WGPUQueue queue() const noexcept { return _context.queue(); }

    [[nodiscard]] const Texture& defaultCheckerTexture() const noexcept { return _renderPipeline.defaultCheckerTexture(); }
    [[nodiscard]] const Texture& defaultGridTexture() const noexcept { return _renderPipeline.defaultGridTexture(); }

private:
    void moveFrom(WebGpuBackend&& rhs) noexcept;

    WebGpuContext _context{};
    WebGpuRenderPipeline _renderPipeline{};
    WebGpuComputePipeline _computePipeline{};

    WGPUColor _clearColor{ 0.08, 0.09, 0.12, 1.0 };
    uint64_t _frameCount = 0;
};

} // namespace corium_sim::renderer
