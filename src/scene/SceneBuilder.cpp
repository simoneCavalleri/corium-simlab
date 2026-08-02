#include "corium_sim/scene/SceneBuilder.hpp"
#include "corium_sim/scene/UrdfLoader.hpp"
#include "corium_sim/renderer/MeshLoader.hpp"
#include "corium_sim/Log.hpp"

namespace corium_sim::scene {

using namespace math;

SceneBuilder::SceneBuilder(WGPUDevice device, WGPUQueue queue)
    : _device(device), _queue(queue)
{}

SceneBuilder& SceneBuilder::addGroundGrid(float width, float depth, uint32_t subdivisions)
{
    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = "ground_grid";
    if (_device && _queue) {
        entity.mesh = renderer::Mesh::createPlane(_device, _queue, width, depth, subdivisions);
        entity.texture = renderer::Texture::createGridPattern(_device, _queue, 512, 512);
    }
    entity.material = renderer::Material::Matte({0.3f, 0.35f, 0.40f, 1.0f});
    entity.position = Vec3{0.0f, 0.0f, 0.0f};
    entity.isStatic = true;
    entity.localBounds = renderer::BoundingBox{
        {-width * 0.5f, 0.0f, -depth * 0.5f},
        { width * 0.5f, 0.0f,  depth * 0.5f}
    };

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addCube(
    std::string name,
    const Vec3& position,
    const Vec3& scale,
    const Vec3& rotation,
    const renderer::Material& material,
    bool isStatic,
    bool hasPhysics
)
{
    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    if (_device && _queue) {
        entity.mesh = renderer::Mesh::createCube(_device, _queue, 1.0f);
        entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    }
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.rotation = rotation;
    entity.isStatic = isStatic;
    entity.hasPhysics = hasPhysics;
    entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addSphere(
    std::string name,
    const Vec3& position,
    float radius,
    const Vec3& scale,
    const renderer::Material& material,
    bool isStatic,
    bool hasPhysics
)
{
    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    if (_device && _queue) {
        entity.mesh = renderer::Mesh::createSphere(_device, _queue, radius, 32, 32);
        entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    }
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.isStatic = isStatic;
    entity.hasPhysics = hasPhysics;
    entity.localBounds = renderer::BoundingBox{{-radius, -radius, -radius}, {radius, radius, radius}};

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addPyramid(
    std::string name,
    const Vec3& position,
    float baseWidth,
    float height,
    const renderer::Material& material,
    bool isStatic,
    bool hasPhysics
)
{
    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    if (_device && _queue) {
        entity.mesh = renderer::Mesh::createPyramid(_device, _queue, baseWidth, height);
        entity.texture = renderer::Texture::createGridPattern(_device, _queue, 256, 256);
    }
    entity.material = material;
    entity.position = position;
    entity.isStatic = isStatic;
    entity.hasPhysics = hasPhysics;
    float w = baseWidth * 0.5f;
    entity.localBounds = renderer::BoundingBox{{-w, 0.0f, -w}, {w, height, w}};

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addModel(
    std::string name,
    const std::string& objFilePath,
    const Vec3& position,
    const Vec3& scale,
    const Vec3& rotation,
    const renderer::Material& material,
    bool isStatic,
    bool hasPhysics
)
{
    renderer::MeshData data = renderer::MeshLoader::parseOBJFile(objFilePath);

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    if (data.success && _device && _queue) {
        entity.mesh.upload(_device, _queue, data.vertices, data.indices);
        entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    }
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.rotation = rotation;
    entity.isStatic = isStatic;
    entity.hasPhysics = hasPhysics;
    entity.localBounds = data.bounds;

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addCylinder(
    std::string name,
    const Vec3& position,
    float radius,
    float height,
    uint32_t segments,
    const Vec3& scale,
    const renderer::Material& material,
    bool isStatic,
    bool hasPhysics
)
{
    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    if (_device && _queue) {
        entity.mesh = renderer::Mesh::createCylinder(_device, _queue, radius, height, segments);
        entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    }
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.isStatic = isStatic;
    entity.hasPhysics = hasPhysics;
    float halfH = height * 0.5f;
    entity.localBounds = renderer::BoundingBox{{-radius, -halfH, -radius}, {radius, halfH, radius}};

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addEntity(SimEntity entity)
{
    if (entity.id == 0) {
        entity.id = _nextEntityId++;
    }
    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addJoint(
    std::string name,
    std::string parentName,
    std::string childName,
    kinematics::JointType type,
    const Vec3& anchor,
    const Vec3& axis,
    float minLimit,
    float maxLimit
)
{
    kinematics::SimJoint joint{};
    joint.id = _nextEntityId++;
    joint.name = name;
    joint.parentName = parentName;
    joint.childName = childName;
    joint.type = type;
    joint.anchor = anchor;
    joint.axis = axis;
    joint.minLimit = minLimit;
    joint.maxLimit = maxLimit;

    if (SimEntity* child = _scene.findEntity(childName)) {
        child->hasPhysics = false;
    }

    _scene.addJoint(std::move(joint));
    return *this;
}

SceneBuilder& SceneBuilder::addURDF(
    const std::string& urdfFilePath,
    const Vec3& basePosition,
    const Vec3& baseScale
)
{
    UrdfLoader::loadURDF(urdfFilePath, _scene, _device, _queue, basePosition, baseScale);
    return *this;
}

SimScene SceneBuilder::build()
{
    CORIUM_LOG_INFO("SceneBuilder", "Finalized 3D simulation environment scene with ",
                    _scene.entityCount(), " entities.");
    return std::move(_scene);
}

} // namespace corium_sim::scene
