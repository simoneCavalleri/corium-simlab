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

/// @brief Core WebGPU Hardware Context (Instance, Adapter, Device, Queue, Surface, Depth Buffer).
class WebGpuContext {
public:
    WebGpuContext() = default;
    ~WebGpuContext();

    WebGpuContext(const WebGpuContext&) = delete;
    WebGpuContext& operator=(const WebGpuContext&) = delete;

    WebGpuContext(WebGpuContext&& rhs) noexcept;
    WebGpuContext& operator=(WebGpuContext&& rhs) noexcept;

    bool initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height) noexcept;
    void resize(uint32_t width, uint32_t height) noexcept;
    void shutdown() noexcept;

    [[nodiscard]] bool isInitialized() const noexcept { return _initialized; }
    [[nodiscard]] uint32_t width() const noexcept { return _width; }
    [[nodiscard]] uint32_t height() const noexcept { return _height; }

    [[nodiscard]] WGPUInstance instance() const noexcept { return _instance; }
    [[nodiscard]] WGPUAdapter adapter() const noexcept { return _adapter; }
    [[nodiscard]] WGPUDevice device() const noexcept { return _device; }
    [[nodiscard]] WGPUQueue queue() const noexcept { return _queue; }
    [[nodiscard]] WGPUSurface surface() const noexcept { return _surface; }
    [[nodiscard]] WGPUTextureFormat surfaceFormat() const noexcept { return _surfaceFormat; }
    [[nodiscard]] WGPUTextureView depthTextureView() const noexcept { return _depthTextureView; }

private:
    void createSurface() noexcept;
    void configureSurface() noexcept;
    void createDepthBuffer() noexcept;
    void moveFrom(WebGpuContext&& rhs) noexcept;

    WGPUInstance _instance = nullptr;
    WGPUAdapter _adapter = nullptr;
    WGPUDevice _device = nullptr;
    WGPUQueue _queue = nullptr;
    WGPUSurface _surface = nullptr;
    WGPUTextureFormat _surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

    WGPUTexture _depthTexture = nullptr;
    WGPUTextureView _depthTextureView = nullptr;

    GLFWwindow* _window = nullptr;
    uint32_t _width = 1280;
    uint32_t _height = 720;
    bool _initialized = false;
};

} // namespace corium_sim::renderer
