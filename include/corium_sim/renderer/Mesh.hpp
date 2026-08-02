#pragma once

#include <cstdint>
#include <string>
#include <vector>

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

namespace corium_sim::renderer {

/// @brief 3D Vertex layout matching WGSL Shader Attributes (Position, Normal, UV, Color).
struct alignas(16) Vertex {
    float position[3];
    float normal[3];
    float uv[2];
    float color[4];
};

/// @brief 3D Mesh GPU Buffer wrapper managing Vertex and Index WebGPU buffers.
class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    Mesh(Mesh&& rhs) noexcept;
    Mesh& operator=(Mesh&& rhs) noexcept;

    bool upload(WGPUDevice device, WGPUQueue queue, const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices);
    void render(WGPURenderPassEncoder passEncoder) const noexcept;
    void destroy() noexcept;

    [[nodiscard]] bool isValid() const noexcept { return _vertexBuffer != nullptr && _indexCount > 0; }
    [[nodiscard]] uint32_t indexCount() const noexcept { return _indexCount; }
    [[nodiscard]] uint32_t vertexCount() const noexcept { return _vertexCount; }

    // 3D Asset Loading APIs
    bool loadFromOBJ(WGPUDevice device, WGPUQueue queue, const std::string& filePath);
    static Mesh createFromOBJ(WGPUDevice device, WGPUQueue queue, const std::string& filePath);

    // Primitive Generators
    static Mesh createCube(WGPUDevice device, WGPUQueue queue, float sideLength = 1.0f);
    static Mesh createSphere(WGPUDevice device, WGPUQueue queue, float radius = 0.5f, uint32_t rings = 24, uint32_t sectors = 24);
    static Mesh createPlane(WGPUDevice device, WGPUQueue queue, float width = 20.0f, float depth = 20.0f, uint32_t gridSubdivisions = 20);
    static Mesh createPyramid(WGPUDevice device, WGPUQueue queue, float baseWidth = 1.0f, float height = 1.5f);
    static Mesh createCylinder(WGPUDevice device, WGPUQueue queue, float radius = 0.5f, float height = 1.0f, uint32_t segments = 24);

private:
    WGPUBuffer _vertexBuffer = nullptr;
    WGPUBuffer _indexBuffer = nullptr;
    uint32_t _vertexCount = 0;
    uint32_t _indexCount = 0;
};

} // namespace corium_sim::renderer
