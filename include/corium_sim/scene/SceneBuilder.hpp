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

    SceneBuilder(const SceneBuilder&) = delete;
    SceneBuilder& operator=(const SceneBuilder&) = delete;

    SceneBuilder(SceneBuilder&&) noexcept = default;
    SceneBuilder& operator=(SceneBuilder&&) noexcept = default;


    /// @brief Add a ground reference grid plane entity to the scene.
    SceneBuilder& addGroundGrid(float width = 50.0f, float depth = 50.0f, uint32_t subdivisions = 50);

    /// @brief Add a 3D Cube primitive entity to the scene with material properties.
    SceneBuilder& addCube(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const math::Vec3& rotation = {0.0f, 0.0f, 0.0f},
        const renderer::Material& material = {},
        bool isStatic = false,
        bool hasPhysics = true
    );

    /// @brief Add a 3D UV Sphere primitive entity to the scene with material properties.
    SceneBuilder& addSphere(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        float radius = 0.5f,
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const renderer::Material& material = {},
        bool isStatic = false,
        bool hasPhysics = true
    );

    /// @brief Add a 3D Pyramid primitive entity to the scene with material properties.
    SceneBuilder& addPyramid(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        float baseWidth = 1.0f,
        float height = 1.5f,
        const renderer::Material& material = {},
        bool isStatic = false,
        bool hasPhysics = true
    );

    /// @brief Load and add a 3D Wavefront OBJ mesh model entity to the scene with material properties.
    SceneBuilder& addModel(
        std::string name,
        const std::string& objFilePath,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const math::Vec3& rotation = {0.0f, 0.0f, 0.0f},
        const renderer::Material& material = {},
        bool isStatic = false,
        bool hasPhysics = true
    );

    /// @brief Add a 3D Cylinder primitive entity to the scene with material properties.
    SceneBuilder& addCylinder(
        std::string name,
        const math::Vec3& position = {0.0f, 0.0f, 0.0f},
        float radius = 0.5f,
        float height = 1.0f,
        uint32_t segments = 24,
        const math::Vec3& scale = {1.0f, 1.0f, 1.0f},
        const renderer::Material& material = {},
        bool isStatic = false,
        bool hasPhysics = true
    );

    /// @brief Add a custom pre-constructed entity to the scene.
    SceneBuilder& addEntity(SimEntity entity);

    /// @brief Add an articulated joint connecting parent and child link entities.
    SceneBuilder& addJoint(
        std::string name,
        std::string parentName,
        std::string childName,
        kinematics::JointType type = kinematics::JointType::Revolute,
        const math::Vec3& anchor = {0.0f, 0.0f, 0.0f},
        const math::Vec3& axis = {0.0f, 1.0f, 0.0f},
        float minLimit = -3.14159f,
        float maxLimit =  3.14159f
    );

    /// @brief Parse and import a URDF (Unified Robot Description Format) XML robot specification file.
    SceneBuilder& addURDF(
        const std::string& urdfFilePath,
        const math::Vec3& basePosition = {0.0f, 0.0f, 0.0f},
        const math::Vec3& baseScale = {1.0f, 1.0f, 1.0f}
    );

    /// @brief Finalize and build the completed SimScene environment.
    [[nodiscard]] SimScene build();

private:
    WGPUDevice _device = nullptr;
    WGPUQueue _queue = nullptr;
    SimScene _scene{};
    uint32_t _nextEntityId = 1;
};

} // namespace corium_sim::scene
