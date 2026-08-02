#pragma once

#include <string>
#include <memory>

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

#include "corium_sim/renderer/Material.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::scene {

/// @brief High-level Fluent C++ Builder API for programmatic 3D agent simulation environment creation.
class SceneBuilder {
public:
    SceneBuilder(WGPUDevice device, WGPUQueue queue);
    ~SceneBuilder() = default;

    /// @brief Add a ground reference grid plane entity to the scene.
    SceneBuilder& addGroundGrid(float width = 50.0f, float depth = 50.0f, uint32_t subdivisions = 50);

    /// @brief Add a 3D Cube primitive entity to the scene with material properties.
    SceneBuilder& addCube(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const math::Vec3& rotation = {0.0f, 0.0f, 0.0f},
        const renderer::Material& material = {}
    );

    /// @brief Add a 3D UV Sphere primitive entity to the scene with material properties.
    SceneBuilder& addSphere(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        float radius = 0.5f,
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const renderer::Material& material = {}
    );

    /// @brief Add a 3D Pyramid primitive entity to the scene with material properties.
    SceneBuilder& addPyramid(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        float baseWidth = 1.0f,
        float height = 1.5f,
        const renderer::Material& material = {}
    );

    /// @brief Load and add a 3D Wavefront OBJ mesh model entity to the scene with material properties.
    SceneBuilder& addModel(
        std::string name,
        const std::string& objFilePath,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const math::Vec3& rotation = {0.0f, 0.0f, 0.0f},
        const renderer::Material& material = {}
    );

    /// @brief Add a custom pre-constructed entity to the scene.
    SceneBuilder& addEntity(SimEntity entity);

    /// @brief Finalize and build the completed SimScene environment.
    [[nodiscard]] SimScene build();

private:
    WGPUDevice _device = nullptr;
    WGPUQueue _queue = nullptr;
    SimScene _scene{};
    uint32_t _nextEntityId = 1;
};

} // namespace corium_sim::scene
