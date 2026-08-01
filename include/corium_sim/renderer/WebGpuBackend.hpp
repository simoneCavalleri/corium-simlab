#pragma once

#include <cstdint>

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

struct GLFWwindow;

namespace corium_sim::renderer {

/// @brief Vertex structure for WebGPU pipeline (Position + Color)
struct alignas(16) Vertex {
    float position[3];
    float color[3];
};

/// @brief WebGPU Rendering Backend implementing native WebGPU pipeline lifecycle.
/// Manages WGPUInstance, WGPUAdapter, WGPUDevice, WGPUQueue, WGPUSurface, and WGPURenderPipeline.
/// Zero dynamic memory allocation during render pass execution.
class WebGpuBackend {
public:
    WebGpuBackend();
    ~WebGpuBackend();

    WebGpuBackend(const WebGpuBackend&) = delete;
    WebGpuBackend& operator=(const WebGpuBackend&) = delete;

    WebGpuBackend(WebGpuBackend&& rhs) noexcept;
    WebGpuBackend& operator=(WebGpuBackend&& rhs) noexcept;

    /// @brief Initialize WebGPU context, Surface, Adapter, Device, Queue, and WGSL Render Pipeline.
    bool initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height);

    /// @brief Set background clear color for render pass.
    void setClearColor(double r, double g, double b, double a = 1.0) noexcept;

    /// @brief Resize WebGPU surface swapchain viewport.
    void resize(uint32_t width, uint32_t height) noexcept;

    /// @brief Execute WebGPU Render Pass: Begin Render Pass, Clear Color Attachment, Submit & Present.
    void renderFrame(double deltaTime) noexcept;

    /// @brief Shutdown WebGPU resources cleanly.
    void shutdown() noexcept;

    [[nodiscard]] bool isInitialized() const noexcept { return _initialized; }
    [[nodiscard]] uint64_t frameCount() const noexcept { return _frameCount; }
    [[nodiscard]] uint32_t width() const noexcept { return _width; }
    [[nodiscard]] uint32_t height() const noexcept { return _height; }
    [[nodiscard]] WGPUInstance instance() const noexcept { return _instance; }
    [[nodiscard]] WGPUAdapter adapter() const noexcept { return _adapter; }
    [[nodiscard]] WGPUDevice device() const noexcept { return _device; }
    [[nodiscard]] WGPUQueue queue() const noexcept { return _queue; }

private:
    void createSurface();
    void configureSurface();
    void moveFrom(WebGpuBackend&& rhs) noexcept;

    WGPUInstance _instance = nullptr;
    WGPUAdapter _adapter = nullptr;
    WGPUDevice _device = nullptr;
    WGPUQueue _queue = nullptr;
    WGPUSurface _surface = nullptr;
    WGPURenderPipeline _pipeline = nullptr;
    GLFWwindow* _window = nullptr;

    uint32_t _width = 1280;
    uint32_t _height = 720;
    uint64_t _frameCount = 0;
    WGPUColor _clearColor{ 0.08, 0.09, 0.12, 1.0 };
    bool _initialized = false;
};

} // namespace corium_sim::renderer
