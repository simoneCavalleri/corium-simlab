#include "corium_sim/renderer/MeshLoader.hpp"

namespace corium_sim::renderer {

Mesh::~Mesh()
{
    destroy();
}

Mesh::Mesh(Mesh&& rhs) noexcept
{
    _vertexBuffer = rhs._vertexBuffer;
    _indexBuffer = rhs._indexBuffer;
    _vertexCount = rhs._vertexCount;
    _indexCount = rhs._indexCount;

    rhs._vertexBuffer = nullptr;
    rhs._indexBuffer = nullptr;
    rhs._vertexCount = 0;
    rhs._indexCount = 0;
}

Mesh& Mesh::operator=(Mesh&& rhs) noexcept
{
    if (this != &rhs) {
        destroy();
        _vertexBuffer = rhs._vertexBuffer;
        _indexBuffer = rhs._indexBuffer;
        _vertexCount = rhs._vertexCount;
        _indexCount = rhs._indexCount;

        rhs._vertexBuffer = nullptr;
        rhs._indexBuffer = nullptr;
        rhs._vertexCount = 0;
        rhs._indexCount = 0;
    }
    return *this;
}

void Mesh::destroy() noexcept
{
    if (_vertexBuffer) {
        wgpuBufferDestroy(_vertexBuffer);
        wgpuBufferRelease(_vertexBuffer);
        _vertexBuffer = nullptr;
    }
    if (_indexBuffer) {
        wgpuBufferDestroy(_indexBuffer);
        wgpuBufferRelease(_indexBuffer);
        _indexBuffer = nullptr;
    }
    _vertexCount = 0;
    _indexCount = 0;
}

bool Mesh::upload(WGPUDevice device, WGPUQueue queue, const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices)
{
    destroy();
    if (!device || !queue || vertices.empty() || indices.empty()) return false;

    _vertexCount = static_cast<uint32_t>(vertices.size());
    _indexCount = static_cast<uint32_t>(indices.size());

    uint64_t vertexBufferSize = sizeof(Vertex) * vertices.size();
    uint64_t indexBufferSize = sizeof(uint32_t) * indices.size();

    // 1. Create Vertex Buffer
    WGPUBufferDescriptor vBufDesc{};
    vBufDesc.size = vertexBufferSize;
    vBufDesc.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
    vBufDesc.mappedAtCreation = false;
    _vertexBuffer = wgpuDeviceCreateBuffer(device, &vBufDesc);

    if (!_vertexBuffer) return false;
    wgpuQueueWriteBuffer(queue, _vertexBuffer, 0, vertices.data(), vertexBufferSize);

    // 2. Create Index Buffer
    WGPUBufferDescriptor iBufDesc{};
    iBufDesc.size = indexBufferSize;
    iBufDesc.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
    iBufDesc.mappedAtCreation = false;
    _indexBuffer = wgpuDeviceCreateBuffer(device, &iBufDesc);

    if (!_indexBuffer) {
        destroy();
        return false;
    }
    wgpuQueueWriteBuffer(queue, _indexBuffer, 0, indices.data(), indexBufferSize);

    return true;
}

void Mesh::render(WGPURenderPassEncoder passEncoder) const noexcept
{
    if (!isValid() || !passEncoder) return;
    wgpuRenderPassEncoderSetVertexBuffer(passEncoder, 0, _vertexBuffer, 0, sizeof(Vertex) * _vertexCount);
    wgpuRenderPassEncoderSetIndexBuffer(passEncoder, _indexBuffer, WGPUIndexFormat_Uint32, 0, sizeof(uint32_t) * _indexCount);
    wgpuRenderPassEncoderDrawIndexed(passEncoder, _indexCount, 1, 0, 0, 0);
}

bool Mesh::loadFromOBJ(WGPUDevice device, WGPUQueue queue, const std::string& filePath)
{
    MeshData data = MeshLoader::parseOBJFile(filePath);
    if (!data.success) return false;
    return upload(device, queue, data.vertices, data.indices);
}

Mesh Mesh::createFromOBJ(WGPUDevice device, WGPUQueue queue, const std::string& filePath)
{
    Mesh mesh;
    mesh.loadFromOBJ(device, queue, filePath);
    return mesh;
}

// -----------------------------------------------------------------------------
// Primitive Geometry Generators
// -----------------------------------------------------------------------------

Mesh Mesh::createCube(WGPUDevice device, WGPUQueue queue, float sideLength)
{
    float s = sideLength * 0.5f;

    std::vector<Vertex> vertices = {
        // Front face (+Z)
        { {-s, -s,  s}, { 0.0f,  0.0f,  1.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s, -s,  s}, { 0.0f,  0.0f,  1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s,  s,  s}, { 0.0f,  0.0f,  1.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s,  s,  s}, { 0.0f,  0.0f,  1.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Back face (-Z)
        { { s, -s, -s}, { 0.0f,  0.0f, -1.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s, -s, -s}, { 0.0f,  0.0f, -1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s,  s, -s}, { 0.0f,  0.0f, -1.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s,  s, -s}, { 0.0f,  0.0f, -1.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Top face (+Y)
        { {-s,  s,  s}, { 0.0f,  1.0f,  0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s,  s,  s}, { 0.0f,  1.0f,  0.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s,  s, -s}, { 0.0f,  1.0f,  0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s,  s, -s}, { 0.0f,  1.0f,  0.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Bottom face (-Y)
        { {-s, -s, -s}, { 0.0f, -1.0f,  0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s, -s, -s}, { 0.0f, -1.0f,  0.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s, -s,  s}, { 0.0f, -1.0f,  0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s, -s,  s}, { 0.0f, -1.0f,  0.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Right face (+X)
        { { s, -s,  s}, { 1.0f,  0.0f,  0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s, -s, -s}, { 1.0f,  0.0f,  0.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s,  s, -s}, { 1.0f,  0.0f,  0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { s,  s,  s}, { 1.0f,  0.0f,  0.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Left face (-X)
        { {-s, -s, -s}, {-1.0f,  0.0f,  0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s, -s,  s}, {-1.0f,  0.0f,  0.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s,  s,  s}, {-1.0f,  0.0f,  0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-s,  s, -s}, {-1.0f,  0.0f,  0.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} }
    };

    std::vector<uint32_t> indices = {
         0,  1,  2,   0,  2,  3, // Front
         4,  5,  6,   4,  6,  7, // Back
         8,  9, 10,   8, 10, 11, // Top
        12, 13, 14,  12, 14, 15, // Bottom
        16, 17, 18,  16, 18, 19, // Right
        20, 21, 22,  20, 22, 23  // Left
    };

    Mesh mesh;
    mesh.upload(device, queue, vertices, indices);
    return mesh;
}

Mesh Mesh::createSphere(WGPUDevice device, WGPUQueue queue, float radius, uint32_t rings, uint32_t sectors)
{
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    float const R = 1.0f / static_cast<float>(rings - 1);
    float const S = 1.0f / static_cast<float>(sectors - 1);
    constexpr float PI = 3.14159265359f;

    for (uint32_t r = 0; r < rings; ++r) {
        for (uint32_t s = 0; s < sectors; ++s) {
            float y = std::sin(-PI * 0.5f + PI * r * R);
            float x = std::cos(2.0f * PI * s * S) * std::sin(PI * r * R);
            float z = std::sin(2.0f * PI * s * S) * std::sin(PI * r * R);

            Vertex v{};
            v.position[0] = x * radius;
            v.position[1] = y * radius;
            v.position[2] = z * radius;

            v.normal[0] = x;
            v.normal[1] = y;
            v.normal[2] = z;

            v.uv[0] = s * S;
            v.uv[1] = r * R;

            v.color[0] = 0.2f + 0.8f * (s * S);
            v.color[1] = 0.4f + 0.6f * (r * R);
            v.color[2] = 0.9f;
            v.color[3] = 1.0f;

            vertices.push_back(v);
        }
    }

    for (uint32_t r = 0; r < rings - 1; ++r) {
        for (uint32_t s = 0; s < sectors - 1; ++s) {
            indices.push_back(r * sectors + s);
            indices.push_back(r * sectors + (s + 1));
            indices.push_back((r + 1) * sectors + (s + 1));

            indices.push_back(r * sectors + s);
            indices.push_back((r + 1) * sectors + (s + 1));
            indices.push_back((r + 1) * sectors + s);
        }
    }

    Mesh mesh;
    mesh.upload(device, queue, vertices, indices);
    return mesh;
}

Mesh Mesh::createPlane(WGPUDevice device, WGPUQueue queue, float width, float depth, uint32_t gridSubdivisions)
{
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    float halfW = width * 0.5f;
    float halfD = depth * 0.5f;
    float dx = width / static_cast<float>(gridSubdivisions);
    float dz = depth / static_cast<float>(gridSubdivisions);

    for (uint32_t z = 0; z <= gridSubdivisions; ++z) {
        for (uint32_t x = 0; x <= gridSubdivisions; ++x) {
            Vertex v{};
            v.position[0] = -halfW + static_cast<float>(x) * dx;
            v.position[1] = 0.0f;
            v.position[2] = -halfD + static_cast<float>(z) * dz;

            v.normal[0] = 0.0f;
            v.normal[1] = 1.0f;
            v.normal[2] = 0.0f;

            v.uv[0] = static_cast<float>(x) / static_cast<float>(gridSubdivisions) * 10.0f;
            v.uv[1] = static_cast<float>(z) / static_cast<float>(gridSubdivisions) * 10.0f;

            v.color[0] = 0.6f;
            v.color[1] = 0.65f;
            v.color[2] = 0.7f;
            v.color[3] = 1.0f;

            vertices.push_back(v);
        }
    }

    uint32_t stride = gridSubdivisions + 1;
    for (uint32_t z = 0; z < gridSubdivisions; ++z) {
        for (uint32_t x = 0; x < gridSubdivisions; ++x) {
            uint32_t topLeft = z * stride + x;
            uint32_t topRight = topLeft + 1;
            uint32_t bottomLeft = (z + 1) * stride + x;
            uint32_t bottomRight = bottomLeft + 1;

            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    Mesh mesh;
    mesh.upload(device, queue, vertices, indices);
    return mesh;
}

Mesh Mesh::createPyramid(WGPUDevice device, WGPUQueue queue, float baseWidth, float height)
{
    float h = height;
    float w = baseWidth * 0.5f;

    std::vector<Vertex> vertices = {
        // Apex
        { { 0.0f,   h,  0.0f}, { 0.0f,  0.8f,  0.6f}, {0.5f, 0.0f}, {1.0f, 0.3f, 0.2f, 1.0f} }, // 0
        { { 0.0f,   h,  0.0f}, { 0.6f,  0.8f,  0.0f}, {0.5f, 0.0f}, {1.0f, 0.3f, 0.2f, 1.0f} }, // 1
        { { 0.0f,   h,  0.0f}, { 0.0f,  0.8f, -0.6f}, {0.5f, 0.0f}, {1.0f, 0.3f, 0.2f, 1.0f} }, // 2
        { { 0.0f,   h,  0.0f}, {-0.6f,  0.8f,  0.0f}, {0.5f, 0.0f}, {1.0f, 0.3f, 0.2f, 1.0f} }, // 3

        // Base corners
        { {-w, 0.0f,  w}, { 0.0f,  0.8f,  0.6f}, {0.0f, 1.0f}, {0.9f, 0.2f, 0.1f, 1.0f} }, // 4 Front-left
        { { w, 0.0f,  w}, { 0.0f,  0.8f,  0.6f}, {1.0f, 1.0f}, {0.9f, 0.2f, 0.1f, 1.0f} }, // 5 Front-right
        { { w, 0.0f, -w}, { 0.0f,  0.8f, -0.6f}, {1.0f, 1.0f}, {0.9f, 0.2f, 0.1f, 1.0f} }, // 6 Back-right
        { {-w, 0.0f, -w}, { 0.0f,  0.8f, -0.6f}, {0.0f, 1.0f}, {0.9f, 0.2f, 0.1f, 1.0f} }, // 7 Back-left
    };

    std::vector<uint32_t> indices = {
        0, 4, 5, // Front
        1, 5, 6, // Right
        2, 6, 7, // Back
        3, 7, 4, // Left
        4, 7, 6, 4, 6, 5 // Base
    };

    Mesh mesh;
    mesh.upload(device, queue, vertices, indices);
    return mesh;
}

} // namespace corium_sim::renderer
