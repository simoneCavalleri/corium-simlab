#pragma once

#include <vector>
#include <string>
#include <optional>

#if __has_include(<webgpu/webgpu.h>)
#include <webgpu/webgpu.h>
#elif __has_include(<webgpu.h>)
#include <webgpu.h>
#elif __has_include("webgpu.h")
#include "webgpu.h"
#endif

#include "corium_sim/scene/SimEntity.hpp"

namespace corium_sim::scene {

class SceneBuilder; // Forward declaration

/// @brief Container for 3D Agent Simulation Scenes managing entities, meshes, and WebGPU resources.
class SimScene {
public:
    SimScene() = default;
    ~SimScene();

    SimScene(const SimScene&) = delete;
    SimScene& operator=(const SimScene&) = delete;

    SimScene(SimScene&& rhs) noexcept;
    SimScene& operator=(SimScene&& rhs) noexcept;

    /// @brief Create a Fluent SceneBuilder for constructing simulation environments.
    static SceneBuilder builder(WGPUDevice device, WGPUQueue queue);

    /// @brief Add entity to scene.
    void addEntity(SimEntity entity);

    /// @brief Destroy and release GPU resources of all entities in scene.
    void destroy() noexcept;

    /// @brief Find entity by name.
    [[nodiscard]] SimEntity* findEntity(const std::string& name) noexcept;

    /// @brief Find entity by ID.
    [[nodiscard]] SimEntity* getEntity(uint32_t id) noexcept;

    /// @brief Calculate combined AABB bounding box for the entire scene.
    [[nodiscard]] renderer::BoundingBox sceneBounds() const noexcept;

    [[nodiscard]] const std::vector<SimEntity>& entities() const noexcept { return _entities; }
    [[nodiscard]] std::vector<SimEntity>& entities() noexcept { return _entities; }
    [[nodiscard]] std::size_t entityCount() const noexcept { return _entities.size(); }

private:
    std::vector<SimEntity> _entities{};
};

} // namespace corium_sim::scene
