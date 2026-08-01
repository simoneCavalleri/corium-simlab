#pragma once

#include <cstdint>
#include <vector>

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

namespace corium_sim::renderer {

/// @brief WebGPU 2D Texture and Sampler wrapper.
class Texture {
public:
    Texture() = default;
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    Texture(Texture&& rhs) noexcept;
    Texture& operator=(Texture&& rhs) noexcept;

    bool createFromPixels(WGPUDevice device, WGPUQueue queue, uint32_t width, uint32_t height, const uint8_t* rgbaPixels);
    void destroy() noexcept;

    [[nodiscard]] bool isValid() const noexcept { return _texture != nullptr && _view != nullptr; }
    [[nodiscard]] WGPUTexture texture() const noexcept { return _texture; }
    [[nodiscard]] WGPUTextureView view() const noexcept { return _view; }
    [[nodiscard]] WGPUSampler sampler() const noexcept { return _sampler; }
    [[nodiscard]] uint32_t width() const noexcept { return _width; }
    [[nodiscard]] uint32_t height() const noexcept { return _height; }

    // Procedural Texture Generators
    static Texture createCheckerboard(WGPUDevice device, WGPUQueue queue, uint32_t width = 256, uint32_t height = 256, uint32_t checkSize = 32);
    static Texture createGridPattern(WGPUDevice device, WGPUQueue queue, uint32_t width = 256, uint32_t height = 256);
    static Texture createSolidColor(WGPUDevice device, WGPUQueue queue, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

private:
    WGPUTexture _texture = nullptr;
    WGPUTextureView _view = nullptr;
    WGPUSampler _sampler = nullptr;
    uint32_t _width = 0;
    uint32_t _height = 0;
};

} // namespace corium_sim::renderer
