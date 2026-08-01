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

static const char* WGSL_SHADER_SOURCE = R"(
struct Uniforms {
    model: mat4x4<f32>,
    viewProj: mat4x4<f32>,
    lightDir: vec4<f32>,
    lightColor: vec4<f32>,
    cameraPos: vec4<f32>,
    ambientColor: vec4<f32>,
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
    let L = normalize(ubo.lightDir.xyz);
    let V = normalize(ubo.cameraPos.xyz - in.worldPos);
    let H = normalize(L + V);

    let texColor = textureSample(textureData, textureSampler, in.uv);
    let baseColor = in.color * texColor;

    let ambient = ubo.ambientColor.rgb * baseColor.rgb;
    let diff = max(dot(N, L), 0.0);
    let diffuse = diff * ubo.lightColor.rgb * baseColor.rgb;
    let spec = pow(max(dot(N, H), 0.0), 32.0);
    let specular = spec * ubo.lightColor.rgb * 0.3;

    let finalColor = ambient + diffuse + specular;
    return vec4<f32>(finalColor, baseColor.a);
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

void WebGpuBackend::drawMesh(const Mesh& mesh, const Texture& texture, const math::Mat4& modelMatrix) noexcept
{
    if (!_inFrame || !_currentPass || !mesh.isValid()) return;

    _currentUbo.model = modelMatrix;
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

void WebGpuBackend::renderFrame(double deltaTime) noexcept
{
    (void)deltaTime;
    // Legacy call shim
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
