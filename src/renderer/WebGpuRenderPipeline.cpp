#include "corium_sim/renderer/WebGpuRenderPipeline.hpp"
#include "corium_sim/renderer/WgslShaders.hpp"
#include "corium_sim/Log.hpp"

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

WebGpuRenderPipeline::~WebGpuRenderPipeline()
{
    shutdown();
}

WebGpuRenderPipeline::WebGpuRenderPipeline(WebGpuRenderPipeline&& rhs) noexcept
{
    moveFrom(std::move(rhs));
}

WebGpuRenderPipeline& WebGpuRenderPipeline::operator=(WebGpuRenderPipeline&& rhs) noexcept
{
    if (this != &rhs) {
        shutdown();
        moveFrom(std::move(rhs));
    }
    return *this;
}

bool WebGpuRenderPipeline::initialize(WGPUDevice device, WGPUQueue queue, WGPUTextureFormat surfaceFormat) noexcept
{
    if (!device || !queue) return false;
    shutdown();

    createPipelineLayout(device);
    createRenderPipeline(device, surfaceFormat);

    _defaultCheckerTex = Texture::createCheckerboard(device, queue, 256, 256, 32);
    _defaultGridTex = Texture::createGridPattern(device, queue, 512, 512);

    _initialized = (_pipeline != nullptr);
    return _initialized;
}

void WebGpuRenderPipeline::shutdown() noexcept
{
    _defaultCheckerTex.destroy();
    _defaultGridTex.destroy();

    clearRenderPools();

    safeDestroyAndRelease<WGPUBuffer, wgpuBufferDestroy, wgpuBufferRelease>(_uniformBuffer);
    safeRelease<WGPURenderPipeline, wgpuRenderPipelineRelease>(_pipeline);
    safeRelease<WGPUPipelineLayout, wgpuPipelineLayoutRelease>(_pipelineLayout);
    safeRelease<WGPUBindGroupLayout, wgpuBindGroupLayoutRelease>(_bindGroupLayout);

    _initialized = false;
}

bool WebGpuRenderPipeline::beginFrame(
    WGPUDevice device,
    WGPUQueue queue,
    WGPUSurface surface,
    WGPUTextureView depthTextureView,
    const Camera& camera,
    WGPUColor clearColor,
    float time
) noexcept
{
    if (!device || !queue || !surface || !depthTextureView || !_pipeline) return false;

    _currentUbo.model = math::Mat4::identity();
    _currentUbo.viewProj = camera.getViewProjectionMatrix();
    _currentUbo.cameraPos = math::Vec4{ camera.getPosition().x, camera.getPosition().y, camera.getPosition().z, 1.0f };
    _currentUbo.lightDir = math::Vec4{ 0.577f, 0.800f, 0.577f, 0.0f };
    _currentUbo.lightColor = math::Vec4{ 1.0f, 0.98f, 0.94f, 1.0f };
    _currentUbo.time = time;

    _currentUboIndex = 0;
    _inFrame = true;

    WGPUCommandEncoderDescriptor encoderDesc{};
    _currentEncoder = wgpuDeviceCreateCommandEncoder(device, &encoderDesc);

    WGPUSurfaceTexture surfaceTexture{};
    wgpuSurfaceGetCurrentTexture(surface, &surfaceTexture);
    if (surfaceTexture.status != WGPUSurfaceGetCurrentTextureStatus_Success || !surfaceTexture.texture) {
        CORIUM_LOG_ERROR("WebGpuRenderPipeline", "Failed to acquire swapchain surface texture.");
        _inFrame = false;
        return false;
    }

    _currentSurfaceTexture = surfaceTexture.texture;

    WGPUTextureViewDescriptor viewDesc{};
    viewDesc.format = wgpuTextureGetFormat(_currentSurfaceTexture);
    viewDesc.dimension = WGPUTextureViewDimension_2D;
    viewDesc.baseMipLevel = 0;
    viewDesc.mipLevelCount = 1;
    viewDesc.baseArrayLayer = 0;
    viewDesc.arrayLayerCount = 1;
    viewDesc.aspect = WGPUTextureAspect_All;

    _currentColorView = wgpuTextureCreateView(_currentSurfaceTexture, &viewDesc);

    WGPURenderPassColorAttachment colorAttachment{};
    colorAttachment.view = _currentColorView;
    colorAttachment.loadOp = WGPULoadOp_Clear;
    colorAttachment.storeOp = WGPUStoreOp_Store;
    colorAttachment.clearValue = clearColor;

    WGPURenderPassDepthStencilAttachment depthAttachment{};
    depthAttachment.view = depthTextureView;
    depthAttachment.depthClearValue = 1.0f;
    depthAttachment.depthLoadOp = WGPULoadOp_Clear;
    depthAttachment.depthStoreOp = WGPUStoreOp_Store;
    depthAttachment.stencilClearValue = 0;
    depthAttachment.stencilLoadOp = WGPULoadOp_Undefined;
    depthAttachment.stencilStoreOp = WGPUStoreOp_Undefined;

    WGPURenderPassDescriptor passDesc{};
    passDesc.colorAttachmentCount = 1;
    passDesc.colorAttachments = &colorAttachment;
    passDesc.depthStencilAttachment = &depthAttachment;

    _currentPass = wgpuCommandEncoderBeginRenderPass(_currentEncoder, &passDesc);
    wgpuRenderPassEncoderSetPipeline(_currentPass, _pipeline);

    return true;
}

void WebGpuRenderPipeline::drawMesh(
    WGPUQueue queue,
    const Mesh& mesh,
    const Texture& texture,
    const math::Mat4& modelMatrix,
    const Material& material
) noexcept
{
    if (!_inFrame || !_currentPass || !mesh.isValid()) return;

    _currentUbo.model = modelMatrix;
    _currentUbo.albedoColor = material.albedo;
    _currentUbo.materialParams = math::Vec4{material.metallic, material.roughness, material.emissive, 0.0f};

    WGPUBuffer ubo = getOrCreateUboBuffer(nullptr, _currentUboIndex);
    wgpuQueueWriteBuffer(queue, ubo, 0, &_currentUbo, sizeof(UniformBufferObject));

    const auto& tex = texture.isValid() ? texture : _defaultCheckerTex;
    WGPUBindGroup bindGroup = getOrCreateBindGroup(nullptr, _currentUboIndex, tex, ubo);
    _currentUboIndex++;

    wgpuRenderPassEncoderSetBindGroup(_currentPass, 0, bindGroup, 0, nullptr);
    mesh.render(_currentPass);
}

void WebGpuRenderPipeline::endFrame(WGPUQueue queue) noexcept
{
    if (!_inFrame || !_currentPass || !_currentEncoder) return;

    wgpuRenderPassEncoderEnd(_currentPass);
    safeRelease<WGPURenderPassEncoder, wgpuRenderPassEncoderRelease>(_currentPass);

    WGPUCommandBufferDescriptor cmdBufferDesc{};
    WGPUCommandBuffer cmdBuffer = wgpuCommandEncoderFinish(_currentEncoder, &cmdBufferDesc);
    wgpuQueueSubmit(queue, 1, &cmdBuffer);

    safeRelease<WGPUCommandBuffer, wgpuCommandBufferRelease>(cmdBuffer);
    safeRelease<WGPUCommandEncoder, wgpuCommandEncoderRelease>(_currentEncoder);
    safeRelease<WGPUTextureView, wgpuTextureViewRelease>(_currentColorView);
    safeRelease<WGPUTexture, wgpuTextureRelease>(_currentSurfaceTexture);

    _inFrame = false;
}

void WebGpuRenderPipeline::createPipelineLayout(WGPUDevice device) noexcept
{
    WGPUBindGroupLayoutEntry entries[3]{};

    entries[0].binding = 0;
    entries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
    entries[0].buffer.type = WGPUBufferBindingType_Uniform;
    entries[0].buffer.hasDynamicOffset = false;
    entries[0].buffer.minBindingSize = sizeof(UniformBufferObject);

    entries[1].binding = 1;
    entries[1].visibility = WGPUShaderStage_Fragment;
    entries[1].sampler.type = WGPUSamplerBindingType_Filtering;

    entries[2].binding = 2;
    entries[2].visibility = WGPUShaderStage_Fragment;
    entries[2].texture.sampleType = WGPUTextureSampleType_Float;
    entries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
    entries[2].texture.multisampled = false;

    WGPUBindGroupLayoutDescriptor bindGroupLayoutDesc{};
    bindGroupLayoutDesc.entryCount = 3;
    bindGroupLayoutDesc.entries = entries;
    _bindGroupLayout = wgpuDeviceCreateBindGroupLayout(device, &bindGroupLayoutDesc);

    WGPUPipelineLayoutDescriptor pipelineLayoutDesc{};
    pipelineLayoutDesc.bindGroupLayoutCount = 1;
    pipelineLayoutDesc.bindGroupLayouts = &_bindGroupLayout;
    _pipelineLayout = wgpuDeviceCreatePipelineLayout(device, &pipelineLayoutDesc);
}

void WebGpuRenderPipeline::createRenderPipeline(WGPUDevice device, WGPUTextureFormat surfaceFormat) noexcept
{
    WGPUShaderModuleWGSLDescriptor wgslDesc{};
    wgslDesc.chain.sType = WGPUSType_ShaderModuleWGSLDescriptor;
    wgslDesc.code = WGSL_PBR_SHADER_SOURCE;

    WGPUShaderModuleDescriptor shaderDesc{};
    shaderDesc.nextInChain = &wgslDesc.chain;
    WGPUShaderModule shaderModule = wgpuDeviceCreateShaderModule(device, &shaderDesc);

    WGPUVertexAttribute attributes[4]{};
    attributes[0].format = WGPUVertexFormat_Float32x3;
    attributes[0].offset = offsetof(Vertex, position);
    attributes[0].shaderLocation = 0;

    attributes[1].format = WGPUVertexFormat_Float32x3;
    attributes[1].offset = offsetof(Vertex, normal);
    attributes[1].shaderLocation = 1;

    attributes[2].format = WGPUVertexFormat_Float32x2;
    attributes[2].offset = offsetof(Vertex, uv);
    attributes[2].shaderLocation = 2;

    attributes[3].format = WGPUVertexFormat_Float32x4;
    attributes[3].offset = offsetof(Vertex, color);
    attributes[3].shaderLocation = 3;

    WGPUVertexBufferLayout vertexBufferLayout{};
    vertexBufferLayout.arrayStride = sizeof(Vertex);
    vertexBufferLayout.stepMode = WGPUVertexStepMode_Vertex;
    vertexBufferLayout.attributeCount = 4;
    vertexBufferLayout.attributes = attributes;

    WGPUColorTargetState colorTarget{};
    colorTarget.format = surfaceFormat;
    colorTarget.writeMask = WGPUColorWriteMask_All;

    WGPUBlendState blendState{};
    blendState.color.operation = WGPUBlendOperation_Add;
    blendState.color.srcFactor = WGPUBlendFactor_SrcAlpha;
    blendState.color.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
    blendState.alpha.operation = WGPUBlendOperation_Add;
    blendState.alpha.srcFactor = WGPUBlendFactor_One;
    blendState.alpha.dstFactor = WGPUBlendFactor_Zero;
    colorTarget.blend = &blendState;

    WGPUFragmentState fragmentState{};
    fragmentState.module = shaderModule;
    fragmentState.entryPoint = "fs_main";
    fragmentState.targetCount = 1;
    fragmentState.targets = &colorTarget;

    WGPUDepthStencilState depthState{};
    depthState.format = WGPUTextureFormat_Depth24Plus;
    depthState.depthWriteEnabled = true;
    depthState.depthCompare = WGPUCompareFunction_Less;

    WGPURenderPipelineDescriptor pipelineDesc{};
    pipelineDesc.layout = _pipelineLayout;
    pipelineDesc.vertex.module = shaderModule;
    pipelineDesc.vertex.entryPoint = "vs_main";
    pipelineDesc.vertex.bufferCount = 1;
    pipelineDesc.vertex.buffers = &vertexBufferLayout;

    pipelineDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
    pipelineDesc.primitive.stripIndexFormat = WGPUIndexFormat_Undefined;
    pipelineDesc.primitive.frontFace = WGPUFrontFace_CCW;
    pipelineDesc.primitive.cullMode = WGPUCullMode_Back;

    pipelineDesc.multisample.count = 1;
    pipelineDesc.multisample.mask = 0xFFFFFFFF;
    pipelineDesc.multisample.alphaToCoverageEnabled = false;

    pipelineDesc.fragment = &fragmentState;
    pipelineDesc.depthStencil = &depthState;

    _pipeline = wgpuDeviceCreateRenderPipeline(device, &pipelineDesc);
    wgpuShaderModuleRelease(shaderModule);
}

void WebGpuRenderPipeline::clearRenderPools() noexcept
{
    for (auto& entry : _bindGroupPool) {
        if (entry.bindGroup) {
            safeRelease<WGPUBindGroup, wgpuBindGroupRelease>(entry.bindGroup);
        }
    }
    _bindGroupPool.clear();

    for (WGPUBuffer buf : _uboPool) {
        if (buf) {
            safeDestroyAndRelease<WGPUBuffer, wgpuBufferDestroy, wgpuBufferRelease>(buf);
        }
    }
    _uboPool.clear();
    _currentUboIndex = 0;
}

WGPUBuffer WebGpuRenderPipeline::getOrCreateUboBuffer(WGPUDevice device, size_t index) noexcept
{
    if (index < _uboPool.size()) {
        return _uboPool[index];
    }
    WGPUBufferDescriptor uboDesc{};
    uboDesc.size = sizeof(UniformBufferObject);
    uboDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    uboDesc.mappedAtCreation = false;
    WGPUBuffer buf = wgpuDeviceCreateBuffer(device, &uboDesc);
    _uboPool.push_back(buf);
    return buf;
}

WGPUBindGroup WebGpuRenderPipeline::getOrCreateBindGroup(WGPUDevice device, size_t index, const Texture& texture, WGPUBuffer uboBuffer) noexcept
{
    const Texture& texToUse = texture.isValid() ? texture : _defaultCheckerTex;
    WGPUTextureView texView = texToUse.view();
    WGPUSampler texSampler = texToUse.sampler();

    if (index < _bindGroupPool.size()) {
        auto& cached = _bindGroupPool[index];
        if (cached.bindGroup && cached.textureView == texView && cached.sampler == texSampler && cached.uboBuffer == uboBuffer) {
            return cached.bindGroup;
        }
        if (cached.bindGroup) {
            safeRelease<WGPUBindGroup, wgpuBindGroupRelease>(cached.bindGroup);
        }
    } else {
        _bindGroupPool.resize(index + 1);
    }

    WGPUBindGroupEntry entries[3]{};
    entries[0].binding = 0;
    entries[0].buffer = uboBuffer;
    entries[0].offset = 0;
    entries[0].size = sizeof(UniformBufferObject);

    entries[1].binding = 1;
    entries[1].sampler = texSampler;

    entries[2].binding = 2;
    entries[2].textureView = texView;

    WGPUBindGroupDescriptor bgDesc{};
    bgDesc.layout = _bindGroupLayout;
    bgDesc.entryCount = 3;
    bgDesc.entries = entries;

    WGPUBindGroup bindGroup = wgpuDeviceCreateBindGroup(device, &bgDesc);
    _bindGroupPool[index] = BindGroupCacheEntry{
        .bindGroup = bindGroup,
        .textureView = texView,
        .sampler = texSampler,
        .uboBuffer = uboBuffer
    };

    return bindGroup;
}

void WebGpuRenderPipeline::moveFrom(WebGpuRenderPipeline&& rhs) noexcept
{
    _bindGroupLayout = rhs._bindGroupLayout;
    _pipelineLayout = rhs._pipelineLayout;
    _pipeline = rhs._pipeline;
    _uniformBuffer = rhs._uniformBuffer;
    _uboPool = std::move(rhs._uboPool);
    _bindGroupPool = std::move(rhs._bindGroupPool);
    _currentUboIndex = rhs._currentUboIndex;
    _defaultCheckerTex = std::move(rhs._defaultCheckerTex);
    _defaultGridTex = std::move(rhs._defaultGridTex);
    _initialized = rhs._initialized;

    rhs._bindGroupLayout = nullptr;
    rhs._pipelineLayout = nullptr;
    rhs._pipeline = nullptr;
    rhs._uniformBuffer = nullptr;
    rhs._uboPool.clear();
    rhs._bindGroupPool.clear();
    rhs._currentUboIndex = 0;
    rhs._initialized = false;
}

} // namespace corium_sim::renderer
