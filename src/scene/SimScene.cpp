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
    _entityIndex[entity.name] = _entities.size();
    _entities.push_back(std::move(entity));
}

void SimScene::addJoint(kinematics::SimJoint joint)
{
    _jointIndex[joint.name] = _joints.size();
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
    auto it = _entityIndex.find(name);
    if (it != _entityIndex.end() && it->second < _entities.size()) {
        if (_entities[it->second].name == name) {
            return &_entities[it->second];
        }
    }
    // Fallback linear scan if index lookup fails or is out of sync
    for (std::size_t i = 0; i < _entities.size(); ++i) {
        if (_entities[i].name == name) {
            _entityIndex[name] = i; // Repair cache index
            return &_entities[i];
        }
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
    auto it = _jointIndex.find(name);
    if (it != _jointIndex.end() && it->second < _joints.size()) {
        return &_joints[it->second];
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
