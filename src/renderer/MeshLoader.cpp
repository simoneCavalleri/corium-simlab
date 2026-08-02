#include "corium_sim/renderer/MeshLoader.hpp"
#include "corium_sim/AssetResolver.hpp"
#include "corium_sim/Log.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <tuple>

namespace corium_sim::renderer {

using namespace math;

MeshData MeshLoader::parseOBJFile(const std::string& filePath)
{
    std::string resolvedPath = AssetResolver::resolve(filePath);

    std::ifstream file(resolvedPath);
    if (!file.is_open()) {
        CORIUM_LOG_ERROR("MeshLoader", "Unable to open OBJ file: ", filePath, " (resolved to: ", resolvedPath, ")");
        return {};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    MeshData data = parseOBJMemory(buffer.str());
    data.name = resolvedPath;
    return data;
}

MeshData MeshLoader::parseOBJMemory(std::string_view content)
{
    MeshData meshData{};
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;

    // Index lookup cache for deduplication: key (posIdx, uvIdx, normIdx) -> unique vertexIndex
    using VertexKey = std::tuple<int, int, int>;
    std::map<VertexKey, uint32_t> uniqueVertices;

    std::istringstream ss{ std::string(content) };
    std::string line;

    bool hasNormalsInObj = false;

    while (std::getline(ss, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream lineSS(line);
        std::string token;
        lineSS >> token;

        if (token == "v") {
            float x, y, z;
            if (lineSS >> x >> y >> z) {
                positions.push_back(Vec3{x, y, z});
                meshData.bounds.expand(Vec3{x, y, z});
            }
        } else if (token == "vn") {
            float x, y, z;
            if (lineSS >> x >> y >> z) {
                normals.push_back(Vec3{x, y, z});
                hasNormalsInObj = true;
            }
        } else if (token == "vt") {
            float u, v;
            if (lineSS >> u >> v) {
                uvs.push_back(Vec2{u, v});
            }
        } else if (token == "f") {
            std::vector<uint32_t> faceIndices;
            std::string faceVertStr;

            while (lineSS >> faceVertStr) {
                int posIdx = 0, uvIdx = 0, normIdx = 0;
                std::stringstream vertSS(faceVertStr);
                std::string vStr, vtStr, vnStr;

                std::getline(vertSS, vStr, '/');
                std::getline(vertSS, vtStr, '/');
                std::getline(vertSS, vnStr, '/');

                if (!vStr.empty()) posIdx = std::stoi(vStr);
                if (!vtStr.empty()) uvIdx = std::stoi(vtStr);
                if (!vnStr.empty()) normIdx = std::stoi(vnStr);

                // Handle 1-based indexing & negative relative offsets
                if (posIdx < 0) posIdx = static_cast<int>(positions.size()) + posIdx + 1;
                if (uvIdx < 0) uvIdx = static_cast<int>(uvs.size()) + uvIdx + 1;
                if (normIdx < 0) normIdx = static_cast<int>(normals.size()) + normIdx + 1;

                VertexKey key{posIdx, uvIdx, normIdx};
                auto it = uniqueVertices.find(key);
                if (it != uniqueVertices.end()) {
                    faceIndices.push_back(it->second);
                } else {
                    Vertex v{};
                    if (posIdx > 0 && posIdx <= static_cast<int>(positions.size())) {
                        v.position[0] = positions[posIdx - 1].x;
                        v.position[1] = positions[posIdx - 1].y;
                        v.position[2] = positions[posIdx - 1].z;
                    }
                    if (normIdx > 0 && normIdx <= static_cast<int>(normals.size())) {
                        v.normal[0] = normals[normIdx - 1].x;
                        v.normal[1] = normals[normIdx - 1].y;
                        v.normal[2] = normals[normIdx - 1].z;
                    }
                    if (uvIdx > 0 && uvIdx <= static_cast<int>(uvs.size())) {
                        v.uv[0] = uvs[uvIdx - 1].x;
                        v.uv[1] = uvs[uvIdx - 1].y;
                    }

                    // Default vertex color (light gray/blue tint for 3D engine visualization)
                    v.color[0] = 0.75f;
                    v.color[1] = 0.80f;
                    v.color[2] = 0.85f;
                    v.color[3] = 1.0f;

                    uint32_t newIdx = static_cast<uint32_t>(meshData.vertices.size());
                    meshData.vertices.push_back(v);
                    uniqueVertices[key] = newIdx;
                    faceIndices.push_back(newIdx);
                }
            }

            // Triangulate polygonal faces using fan triangulation
            if (faceIndices.size() >= 3) {
                for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                    meshData.indices.push_back(faceIndices[0]);
                    meshData.indices.push_back(faceIndices[i]);
                    meshData.indices.push_back(faceIndices[i + 1]);
                }
            }
        }
    }

    if (!hasNormalsInObj) {
        generateNormalsIfMissing(meshData);
    }

    meshData.success = !meshData.vertices.empty() && !meshData.indices.empty();
    if (meshData.success) {
        CORIUM_LOG_INFO("MeshLoader", "Successfully parsed 3D OBJ model: ",
                        meshData.vertices.size(), " vertices, ",
                        meshData.indices.size() / 3, " triangles.");
    }

    return meshData;
}

void MeshLoader::generateNormalsIfMissing(MeshData& data)
{
    // Accumulate face normals into vertices
    for (size_t i = 0; i + 2 < data.indices.size(); i += 3) {
        uint32_t i0 = data.indices[i];
        uint32_t i1 = data.indices[i + 1];
        uint32_t i2 = data.indices[i + 2];

        Vec3 p0{data.vertices[i0].position[0], data.vertices[i0].position[1], data.vertices[i0].position[2]};
        Vec3 p1{data.vertices[i1].position[0], data.vertices[i1].position[1], data.vertices[i1].position[2]};
        Vec3 p2{data.vertices[i2].position[0], data.vertices[i2].position[1], data.vertices[i2].position[2]};

        Vec3 edge1 = p1 - p0;
        Vec3 edge2 = p2 - p0;
        Vec3 fn = Vec3::cross(edge1, edge2);

        data.vertices[i0].normal[0] += fn.x;
        data.vertices[i0].normal[1] += fn.y;
        data.vertices[i0].normal[2] += fn.z;

        data.vertices[i1].normal[0] += fn.x;
        data.vertices[i1].normal[1] += fn.y;
        data.vertices[i1].normal[2] += fn.z;

        data.vertices[i2].normal[0] += fn.x;
        data.vertices[i2].normal[1] += fn.y;
        data.vertices[i2].normal[2] += fn.z;
    }

    // Normalize accumulated vertex normals
    for (auto& v : data.vertices) {
        Vec3 n{v.normal[0], v.normal[1], v.normal[2]};
        n = n.normalized();
        v.normal[0] = n.x;
        v.normal[1] = n.y;
        v.normal[2] = n.z;
    }
}

} // namespace corium_sim::renderer
