#include "corium_sim/renderer/WebGpuComputePipeline.hpp"
#include "corium_sim/scene/SimScene.hpp"
#include "corium_sim/Log.hpp"

#include <algorithm>
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

WebGpuComputePipeline::~WebGpuComputePipeline()
{
    shutdown();
}

WebGpuComputePipeline::WebGpuComputePipeline(WebGpuComputePipeline&& rhs) noexcept
{
    moveFrom(std::move(rhs));
}

WebGpuComputePipeline& WebGpuComputePipeline::operator=(WebGpuComputePipeline&& rhs) noexcept
{
    if (this != &rhs) {
        shutdown();
        moveFrom(std::move(rhs));
    }
    return *this;
}

bool WebGpuComputePipeline::initialize(WGPUDevice device) noexcept
{
    if (!device) return false;
    shutdown();

    static const char* computeShaderSource = R"(
struct Ray {
    origin: vec4<f32>,
    dir: vec4<f32>,
};

struct AABB {
    minBounds: vec4<f32>,
    maxBounds: vec4<f32>,
};

@group(0) @binding(0) var<storage, read> rays: array<Ray>;
@group(0) @binding(1) var<storage, read> obstacles: array<AABB>;
@group(0) @binding(2) var<storage, read_write> distances: array<f32>;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let idx = global_id.x;
    if (idx >= arrayLength(&rays)) { return; }

    let r_orig = rays[idx].origin.xyz;
    let r_dir = rays[idx].dir.xyz;
    var min_t: f32 = rays[idx].origin.w;

    let num_obstacles = arrayLength(&obstacles);
    for (var i: u32 = 0u; i < num_obstacles; i += 1u) {
        let b_min = obstacles[i].minBounds.xyz;
        let b_max = obstacles[i].maxBounds.xyz;

        var tmin: f32 = 0.0;
        var tmax: f32 = min_t;

        for (var d: u32 = 0u; d < 3u; d += 1u) {
            let dir_d = select(r_dir[d], 1e-6, abs(r_dir[d]) < 1e-6);
            let invD = 1.0 / dir_d;
            var t0 = (b_min[d] - r_orig[d]) * invD;
            var t1 = (b_max[d] - r_orig[d]) * invD;

            if (invD < 0.0) {
                let tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            tmin = max(tmin, t0);
            tmax = min(tmax, t1);
        }

        if (tmax > tmin && tmin > 0.001 && tmin < min_t) {
            min_t = tmin;
        }
    }

    distances[idx] = min_t;
}
)";

    WGPUShaderModuleWGSLDescriptor wgslDesc{};
    wgslDesc.chain.sType = WGPUSType_ShaderModuleWGSLDescriptor;
    wgslDesc.code = computeShaderSource;

    WGPUShaderModuleDescriptor shaderDesc{};
    shaderDesc.nextInChain = &wgslDesc.chain;
    WGPUShaderModule computeModule = wgpuDeviceCreateShaderModule(device, &shaderDesc);

    WGPUBindGroupLayoutEntry entries[3]{};
    entries[0].binding = 0;
    entries[0].visibility = WGPUShaderStage_Compute;
    entries[0].buffer.type = WGPUBufferBindingType_ReadOnlyStorage;

    entries[1].binding = 1;
    entries[1].visibility = WGPUShaderStage_Compute;
    entries[1].buffer.type = WGPUBufferBindingType_ReadOnlyStorage;

    entries[2].binding = 2;
    entries[2].visibility = WGPUShaderStage_Compute;
    entries[2].buffer.type = WGPUBufferBindingType_Storage;

    WGPUBindGroupLayoutDescriptor bglDesc{};
    bglDesc.entryCount = 3;
    bglDesc.entries = entries;
    _computeBindGroupLayout = wgpuDeviceCreateBindGroupLayout(device, &bglDesc);

    WGPUPipelineLayoutDescriptor plDesc{};
    plDesc.bindGroupLayoutCount = 1;
    plDesc.bindGroupLayouts = &_computeBindGroupLayout;
    WGPUPipelineLayout pipelineLayout = wgpuDeviceCreatePipelineLayout(device, &plDesc);

    WGPUComputePipelineDescriptor pipelineDesc{};
    pipelineDesc.layout = pipelineLayout;
    pipelineDesc.compute.module = computeModule;
    pipelineDesc.compute.entryPoint = "main";

    _computePipeline = wgpuDeviceCreateComputePipeline(device, &pipelineDesc);

    wgpuPipelineLayoutRelease(pipelineLayout);
    wgpuShaderModuleRelease(computeModule);

    _initialized = (_computePipeline != nullptr);
    return _initialized;
}

void WebGpuComputePipeline::shutdown() noexcept
{
    safeRelease<WGPUComputePipeline, wgpuComputePipelineRelease>(_computePipeline);
    safeRelease<WGPUBindGroupLayout, wgpuBindGroupLayoutRelease>(_computeBindGroupLayout);
    _initialized = false;
}

bool WebGpuComputePipeline::computeLidarRaycast(
    WGPUDevice device,
    WGPUQueue queue,
    const std::vector<math::Vec3>& rayOrigins,
    const std::vector<math::Vec3>& rayDirections,
    const scene::SimScene& scene,
    float maxDistance,
    std::vector<float>& outDistances
) noexcept
{
    if (!device || !queue || rayOrigins.empty() || rayOrigins.size() != rayDirections.size()) {
        return false;
    }

    if (!_initialized && !initialize(device)) {
        return false;
    }

    uint32_t numRays = static_cast<uint32_t>(rayOrigins.size());
    outDistances.resize(numRays, maxDistance);

    struct GpuAABB {
        float minB[4];
        float maxB[4];
    };
    std::vector<GpuAABB> gpuAabbs;
    for (const auto& entity : scene.entities()) {
        if (!entity.hasPhysics) continue;
        renderer::BoundingBox wb = entity.worldBounds();
        gpuAabbs.push_back({
            {wb.min.x, wb.min.y, wb.min.z, 0.0f},
            {wb.max.x, wb.max.y, wb.max.z, 0.0f}
        });
    }

    if (gpuAabbs.empty()) {
        std::fill(outDistances.begin(), outDistances.end(), maxDistance);
        return true;
    }

    struct GpuRay {
        float orig[4];
        float dir[4];
    };
    std::vector<GpuRay> gpuRays(numRays);
    for (uint32_t i = 0; i < numRays; ++i) {
        gpuRays[i] = {
            {rayOrigins[i].x, rayOrigins[i].y, rayOrigins[i].z, maxDistance},
            {rayDirections[i].x, rayDirections[i].y, rayDirections[i].z, 0.0f}
        };
    }

    uint64_t rayBufferSize = gpuRays.size() * sizeof(GpuRay);
    uint64_t aabbBufferSize = gpuAabbs.size() * sizeof(GpuAABB);
    uint64_t distBufferSize = numRays * sizeof(float);

    WGPUBufferDescriptor bufDesc{};
    bufDesc.usage = WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst;

    bufDesc.size = rayBufferSize;
    WGPUBuffer rayBuf = wgpuDeviceCreateBuffer(device, &bufDesc);
    wgpuQueueWriteBuffer(queue, rayBuf, 0, gpuRays.data(), rayBufferSize);

    bufDesc.size = aabbBufferSize;
    WGPUBuffer aabbBuf = wgpuDeviceCreateBuffer(device, &bufDesc);
    wgpuQueueWriteBuffer(queue, aabbBuf, 0, gpuAabbs.data(), aabbBufferSize);

    bufDesc.usage = WGPUBufferUsage_Storage | WGPUBufferUsage_CopySrc | WGPUBufferUsage_CopyDst;
    bufDesc.size = distBufferSize;
    WGPUBuffer distBuf = wgpuDeviceCreateBuffer(device, &bufDesc);

    bufDesc.usage = WGPUBufferUsage_MapRead | WGPUBufferUsage_CopyDst;
    bufDesc.size = distBufferSize;
    WGPUBuffer stagingBuf = wgpuDeviceCreateBuffer(device, &bufDesc);

    WGPUBindGroupEntry entries[3]{};
    entries[0].binding = 0; entries[0].buffer = rayBuf; entries[0].size = rayBufferSize;
    entries[1].binding = 1; entries[1].buffer = aabbBuf; entries[1].size = aabbBufferSize;
    entries[2].binding = 2; entries[2].buffer = distBuf; entries[2].size = distBufferSize;

    WGPUBindGroupDescriptor bgDesc{};
    bgDesc.layout = _computeBindGroupLayout;
    bgDesc.entryCount = 3;
    bgDesc.entries = entries;
    WGPUBindGroup bindGroup = wgpuDeviceCreateBindGroup(device, &bgDesc);

    WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device, nullptr);
    WGPUComputePassEncoder pass = wgpuCommandEncoderBeginComputePass(encoder, nullptr);
    wgpuComputePassEncoderSetPipeline(pass, _computePipeline);
    wgpuComputePassEncoderSetBindGroup(pass, 0, bindGroup, 0, nullptr);

    uint32_t workgroups = (numRays + 63) / 64;
    wgpuComputePassEncoderDispatchWorkgroups(pass, workgroups, 1, 1);
    wgpuComputePassEncoderEnd(pass);
    wgpuComputePassEncoderRelease(pass);

    wgpuCommandEncoderCopyBufferToBuffer(encoder, distBuf, 0, stagingBuf, 0, distBufferSize);

    WGPUCommandBuffer cmdBuf = wgpuCommandEncoderFinish(encoder, nullptr);
    wgpuQueueSubmit(queue, 1, &cmdBuf);
    wgpuCommandBufferRelease(cmdBuf);
    wgpuCommandEncoderRelease(encoder);

    struct MapContext { bool done = false; } mapCtx;
    auto mapCallback = [](WGPUBufferMapAsyncStatus status, void* userdata) {
        auto* ctx = static_cast<MapContext*>(userdata);
        if (status == WGPUBufferMapAsyncStatus_Success && ctx) {
            ctx->done = true;
        }
    };

    wgpuBufferMapAsync(stagingBuf, WGPUMapMode_Read, 0, distBufferSize, mapCallback, &mapCtx);
    while (!mapCtx.done) {
        wgpuDevicePoll(device, true, nullptr);
    }

    const float* mapped = static_cast<const float*>(wgpuBufferGetConstMappedRange(stagingBuf, 0, distBufferSize));
    if (mapped) {
        std::copy(mapped, mapped + numRays, outDistances.begin());
        wgpuBufferUnmap(stagingBuf);
    }

    wgpuBindGroupRelease(bindGroup);
    wgpuBufferDestroy(rayBuf); wgpuBufferRelease(rayBuf);
    wgpuBufferDestroy(aabbBuf); wgpuBufferRelease(aabbBuf);
    wgpuBufferDestroy(distBuf); wgpuBufferRelease(distBuf);
    wgpuBufferDestroy(stagingBuf); wgpuBufferRelease(stagingBuf);

    return true;
}

void WebGpuComputePipeline::moveFrom(WebGpuComputePipeline&& rhs) noexcept
{
    _computeBindGroupLayout = rhs._computeBindGroupLayout;
    _computePipeline = rhs._computePipeline;
    _initialized = rhs._initialized;

    rhs._computeBindGroupLayout = nullptr;
    rhs._computePipeline = nullptr;
    rhs._initialized = false;
}

} // namespace corium_sim::renderer
