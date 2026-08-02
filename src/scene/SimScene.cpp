#include "corium_sim/scene/SimScene.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"

namespace corium_sim::scene {

SimScene::~SimScene()
{
    destroy();
}

SimScene::SimScene(SimScene&& rhs) noexcept
    : _entities(std::move(rhs._entities))
    , _joints(std::move(rhs._joints))
{
}

SimScene& SimScene::operator=(SimScene&& rhs) noexcept
{
    if (this != &rhs) {
        destroy();
        _entities = std::move(rhs._entities);
        _joints = std::move(rhs._joints);
    }
    return *this;
}

SceneBuilder SimScene::builder(WGPUDevice device, WGPUQueue queue)
{
    return SceneBuilder(device, queue);
}

void SimScene::addEntity(SimEntity entity)
{
    _entities.push_back(std::move(entity));
}

void SimScene::addJoint(kinematics::SimJoint joint)
{
    _joints.push_back(std::move(joint));
}

void SimScene::destroy() noexcept
{
    for (auto& entity : _entities) {
        entity.mesh.destroy();
        entity.texture.destroy();
    }
    _entities.clear();
    _joints.clear();
}

SimEntity* SimScene::findEntity(const std::string& name) noexcept
{
    for (auto& entity : _entities) {
        if (entity.name == name) return &entity;
    }
    return nullptr;
}

SimEntity* SimScene::getEntity(uint32_t id) noexcept
{
    for (auto& entity : _entities) {
        if (entity.id == id) return &entity;
    }
    return nullptr;
}

kinematics::SimJoint* SimScene::findJoint(const std::string& name) noexcept
{
    for (auto& joint : _joints) {
        if (joint.name == name) return &joint;
    }
    return nullptr;
}

renderer::BoundingBox SimScene::sceneBounds() const noexcept
{
    renderer::BoundingBox bounds{};
    for (const auto& entity : _entities) {
        renderer::BoundingBox wb = entity.worldBounds();
        bounds.expand(wb.min);
        bounds.expand(wb.max);
    }
    return bounds;
}

} // namespace corium_sim::scene
