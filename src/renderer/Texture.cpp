#include "corium_sim/renderer/Texture.hpp"
#include <utility>

namespace corium_sim::renderer {

Texture::~Texture()
{
    destroy();
}

Texture::Texture(Texture&& rhs) noexcept
{
    _texture = rhs._texture;
    _view = rhs._view;
    _sampler = rhs._sampler;
    _width = rhs._width;
    _height = rhs._height;

    rhs._texture = nullptr;
    rhs._view = nullptr;
    rhs._sampler = nullptr;
    rhs._width = 0;
    rhs._height = 0;
}

Texture& Texture::operator=(Texture&& rhs) noexcept
{
    if (this != &rhs) {
        destroy();
        _texture = rhs._texture;
        _view = rhs._view;
        _sampler = rhs._sampler;
        _width = rhs._width;
        _height = rhs._height;

        rhs._texture = nullptr;
        rhs._view = nullptr;
        rhs._sampler = nullptr;
        rhs._width = 0;
        rhs._height = 0;
    }
    return *this;
}

void Texture::destroy() noexcept
{
    if (_sampler) {
        wgpuSamplerRelease(_sampler);
        _sampler = nullptr;
    }
    if (_view) {
        wgpuTextureViewRelease(_view);
        _view = nullptr;
    }
    if (_texture) {
        wgpuTextureDestroy(_texture);
        wgpuTextureRelease(_texture);
        _texture = nullptr;
    }
    _width = 0;
    _height = 0;
}

bool Texture::createFromPixels(WGPUDevice device, WGPUQueue queue, uint32_t width, uint32_t height, const uint8_t* rgbaPixels)
{
    destroy();
    if (!device || !queue || width == 0 || height == 0 || !rgbaPixels) return false;

    _width = width;
    _height = height;

    // 1. Create 2D WGPUTexture
    WGPUTextureDescriptor texDesc{};
    texDesc.dimension = WGPUTextureDimension_2D;
    texDesc.size = WGPUExtent3D{ _width, _height, 1 };
    texDesc.mipLevelCount = 1;
    texDesc.sampleCount = 1;
    texDesc.format = WGPUTextureFormat_RGBA8Unorm;
    texDesc.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;

    _texture = wgpuDeviceCreateTexture(device, &texDesc);
    if (!_texture) return false;

    // 2. Upload RGBA Pixels
    WGPUImageCopyTexture destination{};
    destination.texture = _texture;
    destination.mipLevel = 0;
    destination.origin = WGPUOrigin3D{ 0, 0, 0 };
    destination.aspect = WGPUTextureAspect_All;

    WGPUTextureDataLayout dataLayout{};
    dataLayout.offset = 0;
    dataLayout.bytesPerRow = _width * 4;
    dataLayout.rowsPerImage = _height;

    WGPUExtent3D writeSize{ _width, _height, 1 };

    wgpuQueueWriteTexture(queue, &destination, rgbaPixels, _width * _height * 4, &dataLayout, &writeSize);

    // 3. Create Texture View
    WGPUTextureViewDescriptor viewDesc{};
    viewDesc.format = WGPUTextureFormat_RGBA8Unorm;
    viewDesc.dimension = WGPUTextureViewDimension_2D;
    viewDesc.baseMipLevel = 0;
    viewDesc.mipLevelCount = 1;
    viewDesc.baseArrayLayer = 0;
    viewDesc.arrayLayerCount = 1;
    viewDesc.aspect = WGPUTextureAspect_All;

    _view = wgpuTextureCreateView(_texture, &viewDesc);

    // 4. Create Texture Sampler
    WGPUSamplerDescriptor samplerDesc{};
    samplerDesc.addressModeU = WGPUAddressMode_Repeat;
    samplerDesc.addressModeV = WGPUAddressMode_Repeat;
    samplerDesc.addressModeW = WGPUAddressMode_Repeat;
    samplerDesc.magFilter = WGPUFilterMode_Linear;
    samplerDesc.minFilter = WGPUFilterMode_Linear;
    samplerDesc.mipmapFilter = WGPUMipmapFilterMode_Linear;
    samplerDesc.lodMinClamp = 0.0f;
    samplerDesc.lodMaxClamp = 1.0f;
    samplerDesc.maxAnisotropy = 1;

    _sampler = wgpuDeviceCreateSampler(device, &samplerDesc);

    return _view != nullptr && _sampler != nullptr;
}

// Procedural Generators

Texture Texture::createCheckerboard(WGPUDevice device, WGPUQueue queue, uint32_t width, uint32_t height, uint32_t checkSize)
{
    std::vector<uint8_t> pixels(width * height * 4);

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            bool check = ((x / checkSize) + (y / checkSize)) % 2 == 0;
            uint32_t index = (y * width + x) * 4;

            if (check) {
                pixels[index + 0] = 220; // R
                pixels[index + 1] = 225; // G
                pixels[index + 2] = 230; // B
            } else {
                pixels[index + 0] = 60;  // R
                pixels[index + 1] = 65;  // G
                pixels[index + 2] = 75;  // B
            }
            pixels[index + 3] = 255;     // A
        }
    }

    Texture tex;
    tex.createFromPixels(device, queue, width, height, pixels.data());
    return tex;
}

Texture Texture::createGridPattern(WGPUDevice device, WGPUQueue queue, uint32_t width, uint32_t height)
{
    std::vector<uint8_t> pixels(width * height * 4);

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            bool gridLine = (x % 32 == 0 || y % 32 == 0 || x == 0 || y == 0);
            uint32_t index = (y * width + x) * 4;

            if (gridLine) {
                pixels[index + 0] = 0;   // Cyan grid lines
                pixels[index + 1] = 180;
                pixels[index + 2] = 255;
            } else {
                pixels[index + 0] = 25;  // Dark background
                pixels[index + 1] = 30;
                pixels[index + 2] = 40;
            }
            pixels[index + 3] = 255;
        }
    }

    Texture tex;
    tex.createFromPixels(device, queue, width, height, pixels.data());
    return tex;
}

Texture Texture::createSolidColor(WGPUDevice device, WGPUQueue queue, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    uint8_t pixels[4] = { r, g, b, a };
    Texture tex;
    tex.createFromPixels(device, queue, 1, 1, pixels);
    return tex;
}

} // namespace corium_sim::renderer
