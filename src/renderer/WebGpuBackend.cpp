#include "corium_sim/renderer/WebGpuBackend.hpp"

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

static const char* WGSL_SHADER_SOURCE = R"(
struct Uniforms {
    model: mat4x4<f32>,
    viewProj: mat4x4<f32>,
    lightDir: vec4<f32>,
    lightColor: vec4<f32>,
    cameraPos: vec4<f32>,
    ambientColor: vec4<f32>,
    albedoColor: vec4<f32>,
    materialParams: vec4<f32>, // x=metallic, y=roughness, z=emissive
    time: f32,
};

@group(0) @binding(0) var<uniform> ubo: Uniforms;
@group(0) @binding(1) var textureSampler: sampler;
@group(0) @binding(2) var textureData: texture_2d<f32>;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clipPosition: vec4<f32>,
    @location(0) worldPos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
};

const PI: f32 = 3.14159265359;

fn distributionGGX(N: vec3<f32>, H: vec3<f32>, roughness: f32) -> f32 {
    let a = roughness * roughness;
    let a2 = a * a;
    let NdotH = max(dot(N, H), 0.0);
    let NdotH2 = NdotH * NdotH;
    let num = a2;
    let denom = (NdotH2 * (a2 - 1.0) + 1.0);
    return num / (PI * denom * denom);
}

fn geometrySchlickGGX(NdotV: f32, roughness: f32) -> f32 {
    let r = (roughness + 1.0);
    let k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

fn geometrySmith(N: vec3<f32>, V: vec3<f32>, L: vec3<f32>, roughness: f32) -> f32 {
    let NdotV = max(dot(N, V), 0.0);
    let NdotL = max(dot(N, L), 0.0);
    let ggx2 = geometrySchlickGGX(NdotV, roughness);
    let ggx1 = geometrySchlickGGX(NdotL, roughness);
    return ggx1 * ggx2;
}

fn fresnelSchlick(cosTheta: f32, F0: vec3<f32>) -> vec3<f32> {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let worldPos4 = ubo.model * vec4<f32>(in.position, 1.0);
    out.worldPos = worldPos4.xyz;
    out.clipPosition = ubo.viewProj * worldPos4;
    
    let normalMatrix = mat3x3<f32>(ubo.model[0].xyz, ubo.model[1].xyz, ubo.model[2].xyz);
    out.normal = normalize(normalMatrix * in.normal);
    
    out.uv = in.uv;
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let N = normalize(in.normal);
    let V = normalize(ubo.cameraPos.xyz - in.worldPos);
    let L = normalize(ubo.lightDir.xyz);
    let H = normalize(V + L);

    let texColor = textureSample(textureData, textureSampler, in.uv);
    let albedo = in.color.rgb * ubo.albedoColor.rgb * texColor.rgb;
    let metallic = clamp(ubo.materialParams.x, 0.0, 1.0);
    let roughness = clamp(ubo.materialParams.y, 0.05, 1.0);
    let emissive = ubo.materialParams.z;

    var F0 = vec3<f32>(0.04);
    F0 = mix(F0, albedo, metallic);

    // Cook-Torrance BRDF
    let NDF = distributionGGX(N, H, roughness);
    let G = geometrySmith(N, V, L, roughness);
    let F = fresnelSchlick(max(dot(H, V), 0.0), F0);

    let numerator = NDF * G * F;
    let denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
    let specular = numerator / denominator;

    let kS = F;
    var kD = vec3<f32>(1.0) - kS;
    kD *= 1.0 - metallic;

    let NdotL = max(dot(N, L), 0.0);
    let radiance = ubo.lightColor.rgb;

    let Lo = (kD * albedo / PI + specular) * radiance * NdotL;
    let ambient = ubo.ambientColor.rgb * albedo;
    let emissiveCol = albedo * emissive;

    let color = ambient + Lo + emissiveCol;

    // Tone mapping and gamma correction
    let mapped = color / (color + vec3<f32>(1.0));
    let finalColor = pow(mapped, vec3<f32>(1.0 / 2.2));

    return vec4<f32>(finalColor, in.color.a * texColor.a * ubo.albedoColor.a);
}
)";

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

    std::cout << "[WebGpuBackend] Initializing 3D WebGPU Engine (" << width << "x" << height << ")..." << std::endl;

    // 1. Create WebGPU Instance
    WGPUInstanceDescriptor instanceDesc{};
    _instance = wgpuCreateInstance(&instanceDesc);

    // 2. Create Surface from Native GLFW Window
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
                    std::cout << "[WebGpuBackend] Adapter message: " << message << "\n";
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
                    std::cout << "[WebGpuBackend] Device message: " << message << "\n";
                }
            },
            &_device
        );
    }

    if (!_device) {
        std::cerr << "[WebGpuBackend] Error: Failed to obtain WGPUDevice!\n";
        return false;
    }

    // 5. Get Device Command Queue & Configure Surface
    _queue = wgpuDeviceGetQueue(_device);
    configureSurface();

    // 6. Create Depth Buffer
    createDepthBuffer();

    // 7. Create Uniform Buffer
    WGPUBufferDescriptor uboDesc{};
    uboDesc.size = sizeof(UniformBufferObject);
    uboDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
    uboDesc.mappedAtCreation = false;
    _uniformBuffer = wgpuDeviceCreateBuffer(_device, &uboDesc);

    // 8. Create Default Procedural Textures
    _defaultCheckerTex = Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    _defaultGridTex = Texture::createGridPattern(_device, _queue, 256, 256);

    // 9. Create Pipeline Layout and WGSL Render Pipeline
    createPipelineLayout();
    createRenderPipeline();

    _clearColor = WGPUColor{ 0.08, 0.09, 0.12, 1.0 };
    _initialized = true;

    std::cout << "[WebGpuBackend] 3D WebGPU Renderer Initialized successfully!\n";
    return true;
}

OffscreenTarget WebGpuBackend::createOffscreenTarget(uint32_t width, uint32_t height) noexcept
{
    if (!_device || width == 0 || height == 0) return {};

    OffscreenTarget target{};
    target.width = width;
    target.height = height;

    // 1. Create Offscreen Color Texture
    WGPUTextureDescriptor colorDesc{};
    colorDesc.dimension = WGPUTextureDimension_2D;
    colorDesc.size = WGPUExtent3D{ width, height, 1 };
    colorDesc.mipLevelCount = 1;
    colorDesc.sampleCount = 1;
    colorDesc.format = WGPUTextureFormat_BGRA8Unorm;
    colorDesc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc | WGPUTextureUsage_TextureBinding;

    target.colorTexture = wgpuDeviceCreateTexture(_device, &colorDesc);

    WGPUTextureViewDescriptor colorViewDesc{};
    colorViewDesc.format = WGPUTextureFormat_BGRA8Unorm;
    colorViewDesc.dimension = WGPUTextureViewDimension_2D;
    colorViewDesc.baseMipLevel = 0;
    colorViewDesc.mipLevelCount = 1;
    colorViewDesc.baseArrayLayer = 0;
    colorViewDesc.arrayLayerCount = 1;
    colorViewDesc.aspect = WGPUTextureAspect_All;

    target.colorView = wgpuTextureCreateView(target.colorTexture, &colorViewDesc);

    // 2. Create Offscreen Depth Buffer Texture
    WGPUTextureDescriptor depthDesc{};
    depthDesc.dimension = WGPUTextureDimension_2D;
    depthDesc.size = WGPUExtent3D{ width, height, 1 };
    depthDesc.mipLevelCount = 1;
    depthDesc.sampleCount = 1;
    depthDesc.format = WGPUTextureFormat_Depth24Plus;
    depthDesc.usage = WGPUTextureUsage_RenderAttachment;

    target.depthTexture = wgpuDeviceCreateTexture(_device, &depthDesc);

    WGPUTextureViewDescriptor depthViewDesc{};
    depthViewDesc.format = WGPUTextureFormat_Depth24Plus;
    depthViewDesc.dimension = WGPUTextureViewDimension_2D;
    depthViewDesc.baseMipLevel = 0;
    depthViewDesc.mipLevelCount = 1;
    depthViewDesc.baseArrayLayer = 0;
    depthViewDesc.arrayLayerCount = 1;
    depthViewDesc.aspect = WGPUTextureAspect_DepthOnly;

    target.depthView = wgpuTextureCreateView(target.depthTexture, &depthViewDesc);

    // 3. Create CPU Staging Buffer for Texture Readback
    uint32_t unalignedBytesPerRow = width * 4;
    target.bytesPerRow = (unalignedBytesPerRow + 255) & ~255;
    target.bufferSize = target.bytesPerRow * height;

    WGPUBufferDescriptor bufDesc{};
    bufDesc.size = target.bufferSize;
    bufDesc.usage = WGPUBufferUsage_MapRead | WGPUBufferUsage_CopyDst;
    bufDesc.mappedAtCreation = false;

    target.stagingBuffer = wgpuDeviceCreateBuffer(_device, &bufDesc);
    target.isValid = (target.colorView != nullptr && target.depthView != nullptr && target.stagingBuffer != nullptr);

    return target;
}

bool WebGpuBackend::copyOffscreenToStaging(const OffscreenTarget& target) noexcept
{
    if (!target.isValid || !target.stagingBuffer || !target.colorTexture || !_device || !_queue) {
        return false;
    }

    WGPUCommandEncoderDescriptor encDesc{};
    WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(_device, &encDesc);

    WGPUImageCopyTexture src{};
    src.texture = target.colorTexture;
    src.mipLevel = 0;
    src.origin = WGPUOrigin3D{ 0, 0, 0 };
    src.aspect = WGPUTextureAspect_All;

    WGPUImageCopyBuffer dst{};
    dst.buffer = target.stagingBuffer;
    dst.layout.offset = 0;
    dst.layout.bytesPerRow = target.bytesPerRow;
    dst.layout.rowsPerImage = target.height;

    WGPUExtent3D copySize{ target.width, target.height, 1 };
    wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &copySize);

    WGPUCommandBufferDescriptor cmdDesc{};
    WGPUCommandBuffer cmdBuf = wgpuCommandEncoderFinish(encoder, &cmdDesc);
    wgpuQueueSubmit(_queue, 1, &cmdBuf);

    wgpuCommandBufferRelease(cmdBuf);
    wgpuCommandEncoderRelease(encoder);
    return true;
}

std::vector<uint8_t> WebGpuBackend::readOffscreenPixels(const OffscreenTarget& target) noexcept
{
    if (!copyOffscreenToStaging(target)) return {};

    std::vector<uint8_t> pixels;
    pixels.reserve(target.width * target.height * 4);

    struct MapContext { bool done = false; WGPUBufferMapAsyncStatus status = WGPUBufferMapAsyncStatus_Unknown; };
    MapContext ctx{};

    wgpuBufferMapAsync(
        target.stagingBuffer,
        WGPUMapMode_Read,
        0,
        target.bufferSize,
        [](WGPUBufferMapAsyncStatus status, void* userdata) {
            auto* c = static_cast<MapContext*>(userdata);
            c->status = status;
            c->done = true;
        },
        &ctx
    );

    wgpuDevicePoll(_device, true, nullptr);

    if (ctx.done && ctx.status == WGPUBufferMapAsyncStatus_Success) {
        const uint8_t* mappedData = static_cast<const uint8_t*>(
            wgpuBufferGetConstMappedRange(target.stagingBuffer, 0, target.bufferSize)
        );

        if (mappedData) {
            for (uint32_t y = 0; y < target.height; ++y) {
                const uint8_t* row = mappedData + y * target.bytesPerRow;
                for (uint32_t x = 0; x < target.width; ++x) {
                    uint8_t b = row[x * 4 + 0];
                    uint8_t g = row[x * 4 + 1];
                    uint8_t r = row[x * 4 + 2];
                    uint8_t a = row[x * 4 + 3];
                    pixels.push_back(r);
                    pixels.push_back(g);
                    pixels.push_back(b);
                    pixels.push_back(a);
                }
            }
            wgpuBufferUnmap(target.stagingBuffer);
        }
    }

    return pixels;
}

void WebGpuBackend::destroyOffscreenTarget(OffscreenTarget& target) noexcept
{
    if (target.stagingBuffer) {
        wgpuBufferDestroy(target.stagingBuffer);
        wgpuBufferRelease(target.stagingBuffer);
        target.stagingBuffer = nullptr;
    }
    if (target.colorView) {
        wgpuTextureViewRelease(target.colorView);
        target.colorView = nullptr;
    }
    if (target.colorTexture) {
        wgpuTextureDestroy(target.colorTexture);
        wgpuTextureRelease(target.colorTexture);
        target.colorTexture = nullptr;
    }
    if (target.depthView) {
        wgpuTextureViewRelease(target.depthView);
        target.depthView = nullptr;
    }
    if (target.depthTexture) {
        wgpuTextureDestroy(target.depthTexture);
        wgpuTextureRelease(target.depthTexture);
        target.depthTexture = nullptr;
    }
    target.isValid = false;
}

void WebGpuBackend::setClearColor(double r, double g, double b, double a) noexcept
{
    _clearColor = WGPUColor{ r, g, b, a };
}

void WebGpuBackend::resize(uint32_t width, uint32_t height) noexcept
{
    if (width == 0 || height == 0) return;
    _width = width;
    _height = height;
    configureSurface();
    createDepthBuffer();
}

bool WebGpuBackend::beginFrame(const Camera& camera, float time) noexcept
{
    if (!_initialized || _inFrame || !_surface) return false;

    _frameCount++;
    _inFrame = true;

    // Update Uniform Buffer Object
    _currentUbo.viewProj = camera.getViewProjectionMatrix();
    math::Vec3 camPos = camera.getPosition();
    _currentUbo.cameraPos = math::Vec4{camPos.x, camPos.y, camPos.z, 1.0f};
    _currentUbo.time = time;

    // Create Command Encoder
    WGPUCommandEncoderDescriptor encoderDesc{};
    _currentEncoder = wgpuDeviceCreateCommandEncoder(_device, &encoderDesc);

    WGPUSurfaceTexture surfaceTexture{};
    wgpuSurfaceGetCurrentTexture(_surface, &surfaceTexture);

    if (surfaceTexture.status != WGPUSurfaceGetCurrentTextureStatus_Success || !surfaceTexture.texture) {
        wgpuCommandEncoderRelease(_currentEncoder);
        _currentEncoder = nullptr;
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
    colorAttachment.clearValue = _clearColor;

    WGPURenderPassDepthStencilAttachment depthAttachment{};
    depthAttachment.view = _depthTextureView;
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

WGPUBindGroup WebGpuBackend::createBindGroup(const Texture& texture) noexcept
{
    const Texture& texToUse = texture.isValid() ? texture : _defaultCheckerTex;

    WGPUBindGroupEntry entries[3]{};

    // Binding 0: Uniform Buffer
    entries[0].binding = 0;
    entries[0].buffer = _uniformBuffer;
    entries[0].offset = 0;
    entries[0].size = sizeof(UniformBufferObject);

    // Binding 1: Sampler
    entries[1].binding = 1;
    entries[1].sampler = texToUse.sampler();

    // Binding 2: Texture View
    entries[2].binding = 2;
    entries[2].textureView = texToUse.view();

    WGPUBindGroupDescriptor bgDesc{};
    bgDesc.layout = _bindGroupLayout;
    bgDesc.entryCount = 3;
    bgDesc.entries = entries;

    return wgpuDeviceCreateBindGroup(_device, &bgDesc);
}

void WebGpuBackend::drawMesh(
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

    wgpuQueueWriteBuffer(_queue, _uniformBuffer, 0, &_currentUbo, sizeof(UniformBufferObject));

    WGPUBindGroup bindGroup = createBindGroup(texture);
    wgpuRenderPassEncoderSetBindGroup(_currentPass, 0, bindGroup, 0, nullptr);

    mesh.render(_currentPass);

    _activeBindGroups.push_back(bindGroup);
}

void WebGpuBackend::endFrame() noexcept
{
    if (!_inFrame || !_currentPass || !_currentEncoder) return;

    wgpuRenderPassEncoderEnd(_currentPass);
    wgpuRenderPassEncoderRelease(_currentPass);
    _currentPass = nullptr;

    WGPUCommandBufferDescriptor cmdBufferDesc{};
    WGPUCommandBuffer cmdBuffer = wgpuCommandEncoderFinish(_currentEncoder, &cmdBufferDesc);

    wgpuQueueSubmit(_queue, 1, &cmdBuffer);

    // Release bind groups before present
    for (WGPUBindGroup bg : _activeBindGroups) {
        if (bg) wgpuBindGroupRelease(bg);
    }
    _activeBindGroups.clear();

    // Release ColorView before present — surface texture is owned by the swapchain, do NOT release it
    if (_currentColorView) {
        wgpuTextureViewRelease(_currentColorView);
        _currentColorView = nullptr;
    }
    _currentSurfaceTexture = nullptr; // not owned, just clear the pointer

    wgpuSurfacePresent(_surface);

    wgpuCommandBufferRelease(cmdBuffer);
    wgpuCommandEncoderRelease(_currentEncoder);
    _currentEncoder = nullptr;

    _inFrame = false;
}

void WebGpuBackend::shutdown() noexcept
{
    if (!_initialized) return;

    _defaultCheckerTex.destroy();
    _defaultGridTex.destroy();

    if (_uniformBuffer) {
        wgpuBufferDestroy(_uniformBuffer);
        wgpuBufferRelease(_uniformBuffer);
        _uniformBuffer = nullptr;
    }
    if (_pipeline) {
        wgpuRenderPipelineRelease(_pipeline);
        _pipeline = nullptr;
    }
    if (_pipelineLayout) {
        wgpuPipelineLayoutRelease(_pipelineLayout);
        _pipelineLayout = nullptr;
    }
    if (_bindGroupLayout) {
        wgpuBindGroupLayoutRelease(_bindGroupLayout);
        _bindGroupLayout = nullptr;
    }
    if (_depthTextureView) {
        wgpuTextureViewRelease(_depthTextureView);
        _depthTextureView = nullptr;
    }
    if (_depthTexture) {
        wgpuTextureDestroy(_depthTexture);
        wgpuTextureRelease(_depthTexture);
        _depthTexture = nullptr;
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

    _initialized = false;
    std::cout << "[WebGpuBackend] WebGPU context shutdown complete.\n";
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

void WebGpuBackend::createDepthBuffer()
{
    if (_depthTextureView) {
        wgpuTextureViewRelease(_depthTextureView);
        _depthTextureView = nullptr;
    }
    if (_depthTexture) {
        wgpuTextureDestroy(_depthTexture);
        wgpuTextureRelease(_depthTexture);
        _depthTexture = nullptr;
    }

    if (!_device) return;

    WGPUTextureDescriptor depthDesc{};
    depthDesc.dimension = WGPUTextureDimension_2D;
    depthDesc.size = WGPUExtent3D{ _width, _height, 1 };
    depthDesc.mipLevelCount = 1;
    depthDesc.sampleCount = 1;
    depthDesc.format = WGPUTextureFormat_Depth24Plus;
    depthDesc.usage = WGPUTextureUsage_RenderAttachment;

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

void WebGpuBackend::createPipelineLayout()
{
    WGPUBindGroupLayoutEntry entries[3]{};

    // Binding 0: Uniform Buffer
    entries[0].binding = 0;
    entries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
    entries[0].buffer.type = WGPUBufferBindingType_Uniform;
    entries[0].buffer.hasDynamicOffset = false;
    entries[0].buffer.minBindingSize = sizeof(UniformBufferObject);

    // Binding 1: Sampler
    entries[1].binding = 1;
    entries[1].visibility = WGPUShaderStage_Fragment;
    entries[1].sampler.type = WGPUSamplerBindingType_Filtering;

    // Binding 2: Texture View
    entries[2].binding = 2;
    entries[2].visibility = WGPUShaderStage_Fragment;
    entries[2].texture.sampleType = WGPUTextureSampleType_Float;
    entries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
    entries[2].texture.multisampled = false;

    WGPUBindGroupLayoutDescriptor bglDesc{};
    bglDesc.entryCount = 3;
    bglDesc.entries = entries;

    _bindGroupLayout = wgpuDeviceCreateBindGroupLayout(_device, &bglDesc);

    WGPUPipelineLayoutDescriptor layoutDesc{};
    layoutDesc.bindGroupLayoutCount = 1;
    layoutDesc.bindGroupLayouts = &_bindGroupLayout;

    _pipelineLayout = wgpuDeviceCreatePipelineLayout(_device, &layoutDesc);
}

void WebGpuBackend::createRenderPipeline()
{
    // WGSL Shader Module
    WGPUShaderModuleWGSLDescriptor wgslDesc{};
    wgslDesc.chain.sType = WGPUSType_ShaderModuleWGSLDescriptor;
    wgslDesc.code = WGSL_SHADER_SOURCE;

    WGPUShaderModuleDescriptor shaderDesc{};
    shaderDesc.nextInChain = &wgslDesc.chain;

    WGPUShaderModule shaderModule = wgpuDeviceCreateShaderModule(_device, &shaderDesc);

    // Vertex Layout Attributes
    WGPUVertexAttribute attributes[4]{};

    // Position (Location 0)
    attributes[0].format = WGPUVertexFormat_Float32x3;
    attributes[0].offset = offsetof(Vertex, position);
    attributes[0].shaderLocation = 0;

    // Normal (Location 1)
    attributes[1].format = WGPUVertexFormat_Float32x3;
    attributes[1].offset = offsetof(Vertex, normal);
    attributes[1].shaderLocation = 1;

    // UV (Location 2)
    attributes[2].format = WGPUVertexFormat_Float32x2;
    attributes[2].offset = offsetof(Vertex, uv);
    attributes[2].shaderLocation = 2;

    // Color (Location 3)
    attributes[3].format = WGPUVertexFormat_Float32x4;
    attributes[3].offset = offsetof(Vertex, color);
    attributes[3].shaderLocation = 3;

    WGPUVertexBufferLayout vertexBufferLayout{};
    vertexBufferLayout.arrayStride = sizeof(Vertex);
    vertexBufferLayout.stepMode = WGPUVertexStepMode_Vertex;
    vertexBufferLayout.attributeCount = 4;
    vertexBufferLayout.attributes = attributes;

    // Color Target State
    WGPUColorTargetState colorTarget{};
    colorTarget.format = WGPUTextureFormat_BGRA8Unorm;
    colorTarget.blend = nullptr;
    colorTarget.writeMask = WGPUColorWriteMask_All;

    // Fragment State
    WGPUFragmentState fragmentState{};
    fragmentState.module = shaderModule;
    fragmentState.entryPoint = "fs_main";
    fragmentState.targetCount = 1;
    fragmentState.targets = &colorTarget;

    // Depth Stencil State
    WGPUDepthStencilState depthState{};
    depthState.format = WGPUTextureFormat_Depth24Plus;
    depthState.depthWriteEnabled = true;
    depthState.depthCompare = WGPUCompareFunction_Less;
    depthState.stencilFront.compare = WGPUCompareFunction_Always;
    depthState.stencilBack.compare = WGPUCompareFunction_Always;

    // Render Pipeline Descriptor
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

    _pipeline = wgpuDeviceCreateRenderPipeline(_device, &pipelineDesc);

    wgpuShaderModuleRelease(shaderModule);
}

void WebGpuBackend::moveFrom(WebGpuBackend&& rhs) noexcept
{
    _instance = rhs._instance;
    _adapter = rhs._adapter;
    _device = rhs._device;
    _queue = rhs._queue;
    _surface = rhs._surface;

    _depthTexture = rhs._depthTexture;
    _depthTextureView = rhs._depthTextureView;
    _bindGroupLayout = rhs._bindGroupLayout;
    _pipelineLayout = rhs._pipelineLayout;
    _pipeline = rhs._pipeline;
    _uniformBuffer = rhs._uniformBuffer;

    _defaultCheckerTex = std::move(rhs._defaultCheckerTex);
    _defaultGridTex = std::move(rhs._defaultGridTex);

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
    rhs._depthTexture = nullptr;
    rhs._depthTextureView = nullptr;
    rhs._bindGroupLayout = nullptr;
    rhs._pipelineLayout = nullptr;
    rhs._pipeline = nullptr;
    rhs._uniformBuffer = nullptr;
    rhs._initialized = false;
}

} // namespace corium_sim::renderer
