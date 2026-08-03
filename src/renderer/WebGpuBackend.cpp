#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/scene/SimScene.hpp"
#include "corium_sim/Log.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

#if __has_include(<wgpu.h>)
#include <wgpu.h>
#elif __has_include("wgpu.h")
#include "wgpu.h"
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

bool WebGpuBackend::initialize(GLFWwindow* windowHandle, uint32_t width, uint32_t height) noexcept
{
    if (!_context.initialize(windowHandle, width, height)) {
        return false;
    }

    if (!_renderPipeline.initialize(_context.device(), _context.queue(), _context.surfaceFormat())) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to initialize 3D Render Pipeline.");
        return false;
    }

    if (!_computePipeline.initialize(_context.device())) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to initialize WGSL Compute Pipeline.");
        return false;
    }

    CORIUM_LOG_INFO("WebGpuBackend", "3D WebGPU Renderer Facade Initialized successfully!");
    return true;
}

void WebGpuBackend::setClearColor(double r, double g, double b, double a) noexcept
{
    _clearColor = WGPUColor{ r, g, b, a };
}

void WebGpuBackend::resize(uint32_t width, uint32_t height) noexcept
{
    _context.resize(width, height);
}

bool WebGpuBackend::beginFrame(const Camera& camera, float time) noexcept
{
    bool ok = _renderPipeline.beginFrame(
        _context.device(),
        _context.queue(),
        _context.surface(),
        _context.depthTextureView(),
        camera,
        _clearColor,
        time
    );

    if (ok) {
        _frameCount++;
    }
    return ok;
}

void WebGpuBackend::drawMesh(
    const Mesh& mesh,
    const Texture& texture,
    const math::Mat4& modelMatrix,
    const Material& material
) noexcept
{
    _renderPipeline.drawMesh(_context.queue(), mesh, texture, modelMatrix, material);
}

void WebGpuBackend::endFrame() noexcept
{
    _renderPipeline.endFrame(_context.queue(), _context.surface());
}

void WebGpuBackend::shutdown() noexcept
{
    _computePipeline.shutdown();
    _renderPipeline.shutdown();
    _context.shutdown();
}

bool WebGpuBackend::computeLidarRaycast(
    const std::vector<math::Vec3>& rayOrigins,
    const std::vector<math::Vec3>& rayDirections,
    const scene::SimScene& scene,
    float maxDistance,
    std::vector<float>& outDistances
) noexcept
{
    return _computePipeline.computeLidarRaycast(
        _context.device(),
        _context.queue(),
        rayOrigins,
        rayDirections,
        scene,
        maxDistance,
        outDistances
    );
}

OffscreenTarget WebGpuBackend::createOffscreenTarget(uint32_t width, uint32_t height) noexcept
{
    OffscreenTarget target{};
    if (!_context.device()) return target;

    target.width = width;
    target.height = height;

    uint32_t unpaddedBytesPerRow = width * 4;
    uint32_t align = 256;
    target.bytesPerRow = (unpaddedBytesPerRow + align - 1) & ~(align - 1);
    target.bufferSize = target.bytesPerRow * height;

    WGPUTextureDescriptor colDesc{};
    colDesc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc;
    colDesc.dimension = WGPUTextureDimension_2D;
    colDesc.size = { width, height, 1 };
    colDesc.format = WGPUTextureFormat_RGBA8UnormSrgb;
    colDesc.mipLevelCount = 1;
    colDesc.sampleCount = 1;

    target.colorTexture = wgpuDeviceCreateTexture(_context.device(), &colDesc);
    if (!target.colorTexture) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to create offscreen color texture.");
        return target;
    }

    WGPUTextureViewDescriptor colViewDesc{};
    colViewDesc.format = WGPUTextureFormat_RGBA8UnormSrgb;
    colViewDesc.dimension = WGPUTextureViewDimension_2D;
    colViewDesc.baseMipLevel = 0;
    colViewDesc.mipLevelCount = 1;
    colViewDesc.baseArrayLayer = 0;
    colViewDesc.arrayLayerCount = 1;
    colViewDesc.aspect = WGPUTextureAspect_All;

    target.colorView = wgpuTextureCreateView(target.colorTexture, &colViewDesc);
    if (!target.colorView) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to create offscreen color view.");
        target.isValid = true; // Temporary for cleanup
        destroyOffscreenTarget(target);
        return target;
    }

    WGPUTextureDescriptor depthDesc{};
    depthDesc.usage = WGPUTextureUsage_RenderAttachment;
    depthDesc.dimension = WGPUTextureDimension_2D;
    depthDesc.size = { width, height, 1 };
    depthDesc.format = WGPUTextureFormat_Depth24Plus;
    depthDesc.mipLevelCount = 1;
    depthDesc.sampleCount = 1;

    target.depthTexture = wgpuDeviceCreateTexture(_context.device(), &depthDesc);
    if (!target.depthTexture) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to create offscreen depth texture.");
        target.isValid = true;
        destroyOffscreenTarget(target);
        return target;
    }

    WGPUTextureViewDescriptor depthViewDesc{};
    depthViewDesc.format = WGPUTextureFormat_Depth24Plus;
    depthViewDesc.dimension = WGPUTextureViewDimension_2D;
    depthViewDesc.baseMipLevel = 0;
    depthViewDesc.mipLevelCount = 1;
    depthViewDesc.baseArrayLayer = 0;
    depthViewDesc.arrayLayerCount = 1;
    depthViewDesc.aspect = WGPUTextureAspect_DepthOnly;

    target.depthView = wgpuTextureCreateView(target.depthTexture, &depthViewDesc);
    if (!target.depthView) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to create offscreen depth view.");
        target.isValid = true;
        destroyOffscreenTarget(target);
        return target;
    }

    WGPUBufferDescriptor bufDesc{};
    bufDesc.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead;
    bufDesc.size = target.bufferSize;
    bufDesc.mappedAtCreation = false;

    target.stagingBuffer = wgpuDeviceCreateBuffer(_context.device(), &bufDesc);
    if (!target.stagingBuffer) {
        CORIUM_LOG_ERROR("WebGpuBackend", "Failed to create offscreen staging buffer.");
        target.isValid = true;
        destroyOffscreenTarget(target);
        return target;
    }

    target.isValid = true;
    return target;
}

void WebGpuBackend::renderOffscreen(const OffscreenTarget& target, const Camera& camera, const scene::SimScene& scene) noexcept
{
    if (!target.isValid || !_context.device() || !_context.queue()) return;

    if (_renderPipeline.beginOffscreenFrame(
            _context.device(), 
            _context.queue(), 
            target.colorView, 
            target.depthView, 
            camera, 
            _clearColor, 
            0.0f)) 
    {
        for (const auto& entity : scene.entities()) {
            if (entity.mesh.isValid()) {
                const auto& tex = entity.texture.isValid() ? entity.texture : _renderPipeline.defaultCheckerTexture();
                _renderPipeline.drawMesh(_context.queue(), entity.mesh, tex, entity.transformMatrix(), entity.material);
            }
        }
        _renderPipeline.endOffscreenFrame(_context.queue());
    }

    WGPUCommandEncoderDescriptor encDesc{};
    WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(_context.device(), &encDesc);

    WGPUImageCopyTexture copySrc{};
    copySrc.texture = target.colorTexture;
    copySrc.aspect = WGPUTextureAspect_All;

    WGPUImageCopyBuffer copyDst{};
    copyDst.buffer = target.stagingBuffer;
    copyDst.layout.bytesPerRow = target.bytesPerRow;
    copyDst.layout.rowsPerImage = target.height;

    WGPUExtent3D copySize{ target.width, target.height, 1 };
    wgpuCommandEncoderCopyTextureToBuffer(encoder, &copySrc, &copyDst, &copySize);

    WGPUCommandBuffer cmdBuf = wgpuCommandEncoderFinish(encoder, nullptr);
    wgpuQueueSubmit(_context.queue(), 1, &cmdBuf);

    wgpuCommandBufferRelease(cmdBuf);
    wgpuCommandEncoderRelease(encoder);
}

bool WebGpuBackend::copyOffscreenToStaging(const OffscreenTarget& target) noexcept
{
    return target.isValid;
}

std::vector<uint8_t> WebGpuBackend::readOffscreenPixels(const OffscreenTarget& target) noexcept
{
    std::vector<uint8_t> pixels;
    if (!target.isValid || !_context.device()) return pixels;

    struct MapCtx { bool done = false; } ctx;
    auto callback = [](WGPUBufferMapAsyncStatus status, void* userdata) {
        auto* c = static_cast<MapCtx*>(userdata);
        if (status == WGPUBufferMapAsyncStatus_Success && c) {
            c->done = true;
        }
    };

    wgpuBufferMapAsync(target.stagingBuffer, WGPUMapMode_Read, 0, target.bufferSize, callback, &ctx);
    while (!ctx.done) {
        wgpuDevicePoll(_context.device(), true, nullptr);
    }

    const uint8_t* mappedData = static_cast<const uint8_t*>(wgpuBufferGetConstMappedRange(target.stagingBuffer, 0, target.bufferSize));
    if (mappedData) {
        pixels.resize(target.width * target.height * 4);
        for (uint32_t y = 0; y < target.height; ++y) {
            const uint8_t* rowSrc = mappedData + (y * target.bytesPerRow);
            uint8_t* rowDst = pixels.data() + (y * target.width * 4);
            std::copy(rowSrc, rowSrc + (target.width * 4), rowDst);
        }
        wgpuBufferUnmap(target.stagingBuffer);
    }

    return pixels;
}

void WebGpuBackend::destroyOffscreenTarget(OffscreenTarget& target) noexcept
{
    if (!target.isValid) return;

    safeDestroyAndRelease<WGPUBuffer, wgpuBufferDestroy, wgpuBufferRelease>(target.stagingBuffer);
    safeRelease<WGPUTextureView, wgpuTextureViewRelease>(target.colorView);
    safeDestroyAndRelease<WGPUTexture, wgpuTextureDestroy, wgpuTextureRelease>(target.colorTexture);
    safeRelease<WGPUTextureView, wgpuTextureViewRelease>(target.depthView);
    safeDestroyAndRelease<WGPUTexture, wgpuTextureDestroy, wgpuTextureRelease>(target.depthTexture);

    target.isValid = false;
}

void WebGpuBackend::moveFrom(WebGpuBackend&& rhs) noexcept
{
    _context = std::move(rhs._context);
    _renderPipeline = std::move(rhs._renderPipeline);
    _computePipeline = std::move(rhs._computePipeline);
    _clearColor = rhs._clearColor;
    _frameCount = rhs._frameCount;
}

} // namespace corium_sim::renderer
