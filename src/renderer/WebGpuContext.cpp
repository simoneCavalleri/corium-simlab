#include "corium_sim/renderer/WebGpuContext.hpp"
#include "corium_sim/Log.hpp"

#include <iostream>
#include <utility>

#if __has_include(<wgpu.h>)
#include <wgpu.h>
#elif __has_include("wgpu.h")
#include "wgpu.h"
#endif

#if __has_include(<GLFW/glfw3.h>)
#include <GLFW/glfw3.h>
#if defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_WAYLAND
#include <GLFW/glfw3native.h>
#endif
#endif

namespace corium_sim::renderer {

template <typename T, void (*ReleaseFunc)(T)>
inline void safeRelease(T& handle) noexcept {
    if (handle) {
        ReleaseFunc(handle);
        handle = nullptr;
    }
}

template <typename T, void (*DestroyFunc)(T), void (*ReleaseFunc)(T)>
inline void safeDestroyAndRelease(T& handle) noexcept {
    if (handle) {
        DestroyFunc(handle);
        ReleaseFunc(handle);
        handle = nullptr;
    }
}

WebGpuContext::~WebGpuContext()
{
    shutdown();
}

WebGpuContext::WebGpuContext(WebGpuContext&& rhs) noexcept
{
    moveFrom(std::move(rhs));
}

WebGpuContext& WebGpuContext::operator=(WebGpuContext&& rhs) noexcept
{
    if (this != &rhs) {
        shutdown();
        moveFrom(std::move(rhs));
    }
    return *this;
}

bool WebGpuContext::initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height) noexcept
{
    _width = width;
    _height = height;
    _window = windowHandle;

    CORIUM_LOG_INFO("WebGpuContext", "Initializing WebGPU Hardware Context (", width, "x", height, ")...");

#if __has_include(<GLFW/glfw3.h>)
    if (glfwGetCurrentContext() != nullptr) {
        glfwMakeContextCurrent(nullptr);
    }
#endif

    WGPUInstanceExtras instanceExtras{};
    instanceExtras.chain.sType = static_cast<WGPUSType>(WGPUSType_InstanceExtras);
    instanceExtras.backends = WGPUInstanceBackend_Primary | WGPUInstanceBackend_GL;

    WGPUInstanceDescriptor instanceDesc{};
    instanceDesc.nextInChain = &instanceExtras.chain;
    _instance = wgpuCreateInstance(&instanceDesc);

    if (_window && _instance) {
        createSurface();
    }

    if (_instance) {
        WGPURequestAdapterOptions adapterOpts{};
        adapterOpts.compatibleSurface = _surface;

        wgpuInstanceRequestAdapter(
            _instance,
            &adapterOpts,
            [](WGPURequestAdapterStatus status, WGPUAdapter adapter, const char* message, void* userdata) {
                if (status == WGPURequestAdapterStatus_Success && userdata) {
                    *static_cast<WGPUAdapter*>(userdata) = adapter;
                } else if (message) {
                    CORIUM_LOG_ERROR("WebGpuContext", "Adapter request failed: ", message);
                }
            },
            &_adapter
        );
    }

    if (_adapter) {
        WGPUAdapterProperties props{};
        wgpuAdapterGetProperties(_adapter, &props);
        CORIUM_LOG_INFO("WebGpuContext", "GPU Adapter: ", props.name, " (BackendType=", props.backendType, ")");

        WGPUDeviceDescriptor deviceDesc{};
        wgpuAdapterRequestDevice(
            _adapter,
            &deviceDesc,
            [](WGPURequestDeviceStatus status, WGPUDevice device, const char* message, void* userdata) {
                if (status == WGPURequestDeviceStatus_Success && userdata) {
                    *static_cast<WGPUDevice*>(userdata) = device;
                } else if (message) {
                    CORIUM_LOG_ERROR("WebGpuContext", "Device request failed: ", message);
                }
            },
            &_device
        );
    }

    if (_device) {
        _queue = wgpuDeviceGetQueue(_device);
    }

    if (_surface && _device && _adapter) {
        configureSurface();
    }

    if (_device) {
        createDepthBuffer();
    }

    _initialized = (_device != nullptr && _queue != nullptr);
    if (_initialized) {
        CORIUM_LOG_INFO("WebGpuContext", "WebGPU Context initialized successfully!");
    } else {
        CORIUM_LOG_ERROR("WebGpuContext", "Failed to initialize WebGPU context.");
    }
    return _initialized;
}

void WebGpuContext::resize(uint32_t width, uint32_t height) noexcept
{
    if (width == 0 || height == 0) return;
    _width = width;
    _height = height;

    if (_surface && _device && _adapter) {
        configureSurface();
    }
    if (_device) {
        createDepthBuffer();
    }
}

void WebGpuContext::shutdown() noexcept
{
    if (!_initialized) return;

    safeRelease<WGPUTextureView, wgpuTextureViewRelease>(_depthTextureView);
    safeDestroyAndRelease<WGPUTexture, wgpuTextureDestroy, wgpuTextureRelease>(_depthTexture);
    safeRelease<WGPUQueue, wgpuQueueRelease>(_queue);

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

    _initialized = false;
}

void WebGpuContext::createSurface() noexcept
{
#if defined(__linux__) && defined(GLFW_EXPOSE_NATIVE_X11)
    WGPUSurfaceDescriptorFromXlibWindow x11Desc{};
    x11Desc.chain.sType = WGPUSType_SurfaceDescriptorFromXlibWindow;
    x11Desc.display = glfwGetX11Display();
    x11Desc.window = glfwGetX11Window(_window);

    WGPUSurfaceDescriptor surfaceDesc{};
    surfaceDesc.nextInChain = &x11Desc.chain;

    _surface = wgpuInstanceCreateSurface(_instance, &surfaceDesc);
    if (_surface) {
        CORIUM_LOG_INFO("WebGpuContext", "WGPUSurface created successfully via X11.");
    }
#endif
}

void WebGpuContext::configureSurface() noexcept
{
    if (_surface && _device && _adapter) {
        WGPUSurfaceCapabilities caps{};
        wgpuSurfaceGetCapabilities(_surface, _adapter, &caps);

        _surfaceFormat = WGPUTextureFormat_BGRA8Unorm;
        if (caps.formatCount > 0) {
            _surfaceFormat = caps.formats[0];
        }

        WGPUSurfaceConfiguration config{};
        config.device = _device;
        config.format = _surfaceFormat;
        config.usage = WGPUTextureUsage_RenderAttachment;
        config.alphaMode = caps.alphaModeCount > 0 ? caps.alphaModes[0] : WGPUCompositeAlphaMode_Auto;
        config.width = _width;
        config.height = _height;
        config.presentMode = WGPUPresentMode_Fifo;

        wgpuSurfaceConfigure(_surface, &config);
    }
}

void WebGpuContext::createDepthBuffer() noexcept
{
    safeRelease<WGPUTextureView, wgpuTextureViewRelease>(_depthTextureView);
    safeDestroyAndRelease<WGPUTexture, wgpuTextureDestroy, wgpuTextureRelease>(_depthTexture);

    WGPUTextureDescriptor depthDesc{};
    depthDesc.usage = WGPUTextureUsage_RenderAttachment;
    depthDesc.dimension = WGPUTextureDimension_2D;
    depthDesc.size = { _width, _height, 1 };
    depthDesc.format = WGPUTextureFormat_Depth24Plus;
    depthDesc.mipLevelCount = 1;
    depthDesc.sampleCount = 1;

    _depthTexture = wgpuDeviceCreateTexture(_device, &depthDesc);

    WGPUTextureViewDescriptor viewDesc{};
    viewDesc.format = WGPUTextureFormat_Depth24Plus;
    viewDesc.dimension = WGPUTextureViewDimension_2D;
    viewDesc.baseMipLevel = 0;
    viewDesc.mipLevelCount = 1;
    viewDesc.baseArrayLayer = 0;
    viewDesc.arrayLayerCount = 1;
    viewDesc.aspect = WGPUTextureAspect_DepthOnly;

    _depthTextureView = wgpuTextureCreateView(_depthTexture, &viewDesc);
}

void WebGpuContext::moveFrom(WebGpuContext&& rhs) noexcept
{
    _instance = rhs._instance;
    _adapter = rhs._adapter;
    _device = rhs._device;
    _queue = rhs._queue;
    _surface = rhs._surface;
    _surfaceFormat = rhs._surfaceFormat;
    _depthTexture = rhs._depthTexture;
    _depthTextureView = rhs._depthTextureView;
    _window = rhs._window;
    _width = rhs._width;
    _height = rhs._height;
    _initialized = rhs._initialized;

    rhs._instance = nullptr;
    rhs._adapter = nullptr;
    rhs._device = nullptr;
    rhs._queue = nullptr;
    rhs._surface = nullptr;
    rhs._depthTexture = nullptr;
    rhs._depthTextureView = nullptr;
    rhs._initialized = false;
}

} // namespace corium_sim::renderer
