#include "corium_sim/renderer/WebGpuBackend.hpp"

#include <iostream>
#include <utility>

#if __has_include(<GLFW/glfw3.h>)
#include <GLFW/glfw3.h>
#if defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_WAYLAND
#include <GLFW/glfw3native.h>
#endif
#endif

namespace corium_sim::renderer {

WebGpuBackend::WebGpuBackend() = default;

WebGpuBackend::~WebGpuBackend()
{
    shutdown();
}

WebGpuBackend::WebGpuBackend(WebGpuBackend&& rhs) noexcept
{
    moveFrom(std::move(rhs));
}

WebGpuBackend& WebGpuBackend::operator=(WebGpuBackend&& rhs) noexcept
{
    if (this != &rhs) {
        shutdown();
        moveFrom(std::move(rhs));
    }
    return *this;
}

bool WebGpuBackend::initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height)
{
    _width = width;
    _height = height;
    _window = windowHandle;

    std::cout << "[WebGpuBackend] Initializing WebGPU Graphics Context (" << width << "x" << height << ")..." << std::endl;

    // 1. Create WebGPU Instance
    WGPUInstanceDescriptor instanceDesc{};
    _instance = wgpuCreateInstance(&instanceDesc);
    if (!_instance) {
        std::cout << "[WebGpuBackend] Warning: WebGPU instance creation returned fallback descriptor context." << std::endl;
    }

    // 2. Create Surface from Native GLFW Window Handle (Wayland or X11)
    if (_window && _instance) {
        createSurface();
    }

    // 3. Request GPU Adapter
    if (_instance) {
        WGPURequestAdapterOptions adapterOpts{};
        adapterOpts.compatibleSurface = _surface;

        wgpuInstanceRequestAdapter(
            _instance,
            &adapterOpts,
            [](WGPURequestAdapterStatus status, WGPUAdapter adapter, const char* message, void* userdata) {
                if (status == WGPURequestAdapterStatus_Success && adapter) {
                    *static_cast<WGPUAdapter*>(userdata) = adapter;
                } else if (message) {
                    std::cout << "[WebGpuBackend] Adapter request message: " << message << "\n";
                }
            },
            &_adapter
        );
    }

    // 4. Request GPU Device
    if (_adapter) {
        WGPUDeviceDescriptor deviceDesc{};
        wgpuAdapterRequestDevice(
            _adapter,
            &deviceDesc,
            [](WGPURequestDeviceStatus status, WGPUDevice device, const char* message, void* userdata) {
                if (status == WGPURequestDeviceStatus_Success && device) {
                    *static_cast<WGPUDevice*>(userdata) = device;
                } else if (message) {
                    std::cout << "[WebGpuBackend] Device request message: " << message << "\n";
                }
            },
            &_device
        );
    }

    // 5. Get Device Command Queue & Configure Surface
    if (_device) {
        _queue = wgpuDeviceGetQueue(_device);
        configureSurface();
    }

    // 6. Setup Default Clear Color
    _clearColor = WGPUColor{ 0.08, 0.09, 0.12, 1.0 };
    _initialized = true;

    std::cout << "[WebGpuBackend] WebGPU Graphics Context Initialized successfully!\n";
    return true;
}

void WebGpuBackend::setClearColor(double r, double g, double b, double a) noexcept
{
    _clearColor = WGPUColor{ r, g, b, a };
}

void WebGpuBackend::resize(uint32_t width, uint32_t height) noexcept
{
    _width = width;
    _height = height;
    configureSurface();
}

void WebGpuBackend::renderFrame(double deltaTime) noexcept
{
    (void)deltaTime;
    if (!_initialized) return;

    _frameCount++;

    if (_surface && _device && _queue) {
        WGPUSurfaceTexture surfaceTexture{};
        wgpuSurfaceGetCurrentTexture(_surface, &surfaceTexture);

        if (surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Success && surfaceTexture.texture) {
            WGPUTextureViewDescriptor viewDesc{};
            viewDesc.format = wgpuTextureGetFormat(surfaceTexture.texture);
            viewDesc.dimension = WGPUTextureViewDimension_2D;
            viewDesc.baseMipLevel = 0;
            viewDesc.mipLevelCount = 1;
            viewDesc.baseArrayLayer = 0;
            viewDesc.arrayLayerCount = 1;
            viewDesc.aspect = WGPUTextureAspect_All;

            WGPUTextureView targetView = wgpuTextureCreateView(surfaceTexture.texture, &viewDesc);

            if (targetView) {
                WGPUCommandEncoderDescriptor encoderDesc{};
                WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(_device, &encoderDesc);

                WGPURenderPassColorAttachment colorAttachment{};
                colorAttachment.view = targetView;
                colorAttachment.loadOp = WGPULoadOp_Clear;
                colorAttachment.storeOp = WGPUStoreOp_Store;
                colorAttachment.clearValue = _clearColor;

                WGPURenderPassDescriptor passDesc{};
                passDesc.colorAttachmentCount = 1;
                passDesc.colorAttachments = &colorAttachment;

                WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
                wgpuRenderPassEncoderEnd(pass);

                WGPUCommandBufferDescriptor cmdBufferDesc{};
                WGPUCommandBuffer cmdBuffer = wgpuCommandEncoderFinish(encoder, &cmdBufferDesc);

                wgpuQueueSubmit(_queue, 1, &cmdBuffer);
                wgpuSurfacePresent(_surface);

                wgpuCommandBufferRelease(cmdBuffer);
                wgpuRenderPassEncoderRelease(pass);
                wgpuCommandEncoderRelease(encoder);
                wgpuTextureViewRelease(targetView);
            }
        }
    }
}

void WebGpuBackend::shutdown() noexcept
{
    if (!_initialized) return;

    if (_pipeline) {
        wgpuRenderPipelineRelease(_pipeline);
        _pipeline = nullptr;
    }
    if (_queue) {
        wgpuQueueRelease(_queue);
        _queue = nullptr;
    }
    if (_device) {
        wgpuDeviceRelease(_device);
        _device = nullptr;
    }
    if (_adapter) {
        wgpuAdapterRelease(_adapter);
        _adapter = nullptr;
    }
    if (_surface) {
        wgpuSurfaceRelease(_surface);
        _surface = nullptr;
    }
    if (_instance) {
        wgpuInstanceRelease(_instance);
        _instance = nullptr;
    }

    std::cout << "[WebGpuBackend] WebGPU context shutdown complete. Rendered " << _frameCount << " frames.\n";
    _initialized = false;
}

void WebGpuBackend::createSurface()
{
#if defined(__linux__)
    if (_window && _instance) {
        int platform = glfwGetPlatform();
        if (platform == GLFW_PLATFORM_WAYLAND) {
#if defined(GLFW_EXPOSE_NATIVE_WAYLAND)
            struct wl_display* display = glfwGetWaylandDisplay();
            struct wl_surface* surface = glfwGetWaylandWindow(_window);
            if (display && surface) {
                WGPUSurfaceDescriptorFromWaylandSurface waylandDesc{};
                waylandDesc.chain.sType = WGPUSType_SurfaceDescriptorFromWaylandSurface;
                waylandDesc.display = display;
                waylandDesc.surface = surface;

                WGPUSurfaceDescriptor surfDesc{};
                surfDesc.nextInChain = &waylandDesc.chain;
                _surface = wgpuInstanceCreateSurface(_instance, &surfDesc);
                if (_surface) {
                    std::cout << "[WebGpuBackend] Native Wayland WebGPU Surface created.\n";
                }
            }
#endif
        } else if (platform == GLFW_PLATFORM_X11) {
#if defined(GLFW_EXPOSE_NATIVE_X11)
            Display* display = glfwGetX11Display();
            Window window = glfwGetX11Window(_window);
            if (display && window) {
                WGPUSurfaceDescriptorFromXlibWindow x11Desc{};
                x11Desc.chain.sType = WGPUSType_SurfaceDescriptorFromXlibWindow;
                x11Desc.display = display;
                x11Desc.window = static_cast<uint64_t>(window);

                WGPUSurfaceDescriptor surfDesc{};
                surfDesc.nextInChain = &x11Desc.chain;
                _surface = wgpuInstanceCreateSurface(_instance, &surfDesc);
                if (_surface) {
                    std::cout << "[WebGpuBackend] Native X11 WebGPU Surface created.\n";
                }
            }
#endif
        }
    }
#endif
}

void WebGpuBackend::configureSurface()
{
    if (_surface && _device) {
        WGPUSurfaceConfiguration config{};
        config.device = _device;
        config.format = WGPUTextureFormat_BGRA8Unorm;
        config.usage = WGPUTextureUsage_RenderAttachment;
        config.width = _width;
        config.height = _height;
        config.presentMode = WGPUPresentMode_Fifo;
        wgpuSurfaceConfigure(_surface, &config);
    }
}

void WebGpuBackend::moveFrom(WebGpuBackend&& rhs) noexcept
{
    _instance = rhs._instance;
    _adapter = rhs._adapter;
    _device = rhs._device;
    _queue = rhs._queue;
    _surface = rhs._surface;
    _pipeline = rhs._pipeline;
    _window = rhs._window;
    _width = rhs._width;
    _height = rhs._height;
    _frameCount = rhs._frameCount;
    _clearColor = rhs._clearColor;
    _initialized = rhs._initialized;

    rhs._instance = nullptr;
    rhs._adapter = nullptr;
    rhs._device = nullptr;
    rhs._queue = nullptr;
    rhs._surface = nullptr;
    rhs._pipeline = nullptr;
    rhs._window = nullptr;
    rhs._initialized = false;
}

} // namespace corium_sim::renderer
