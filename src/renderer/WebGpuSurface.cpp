#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/Log.hpp"

#if __has_include(<wgpu.h>)
#include <wgpu.h>
#elif __has_include("wgpu.h")
#include "wgpu.h"
#endif

#if __has_include(<GLFW/glfw3.h>)
#include <GLFW/glfw3.h>
#if defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#elif defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_WAYLAND
#include <GLFW/glfw3native.h>
#endif
#endif

namespace corium_sim::renderer {

void WebGpuBackend::createSurface()
{
    if (!_window || !_instance) return;

#if defined(_WIN32)
    HWND hwnd = glfwGetWin32Window(_window);
    HINSTANCE hinstance = GetModuleHandle(nullptr);
    if (hwnd) {
        WGPUSurfaceDescriptorFromWindowsHWND winDesc{};
        winDesc.chain.sType = WGPUSType_SurfaceDescriptorFromWindowsHWND;
        winDesc.hinstance = hinstance;
        winDesc.hwnd = hwnd;

        WGPUSurfaceDescriptor surfDesc{};
        surfDesc.nextInChain = &winDesc.chain;
        _surface = wgpuInstanceCreateSurface(_instance, &surfDesc);
    }
#elif defined(__linux__)
    int platform = glfwGetPlatform();
    CORIUM_LOG_INFO("WebGpuBackend", "GLFW Platform: ", (platform == GLFW_PLATFORM_WAYLAND ? "Wayland" : (platform == GLFW_PLATFORM_X11 ? "X11" : "Unknown")));
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
        }
#endif
    }
#endif

    if (!_surface) {
        CORIUM_LOG_WARN("WebGpuBackend", "WGPUSurface not created (running in offscreen / headless mode).");
    } else {
        CORIUM_LOG_INFO("WebGpuBackend", "WGPUSurface created successfully.");
    }
}

} // namespace corium_sim::renderer
