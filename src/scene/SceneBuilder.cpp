#include "corium_sim/scene/SceneBuilder.hpp"
#include "corium_sim/renderer/MeshLoader.hpp"
#include "corium_sim/Log.hpp"

namespace corium_sim::scene {

using namespace math;

SceneBuilder::SceneBuilder(WGPUDevice device, WGPUQueue queue)
    : _device(device), _queue(queue)
{}

SceneBuilder& SceneBuilder::addGroundGrid(float width, float depth, uint32_t subdivisions)
{
    if (!_device || !_queue) return *this;

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = "ground_grid";
    entity.mesh = renderer::Mesh::createPlane(_device, _queue, width, depth, subdivisions);
    entity.texture = renderer::Texture::createGridPattern(_device, _queue, 512, 512);
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
    const renderer::Material& material
)
{
    if (!_device || !_queue) return *this;

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    entity.mesh = renderer::Mesh::createCube(_device, _queue, 1.0f);
    entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.rotation = rotation;
    entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addSphere(
    std::string name,
    const Vec3& position,
    float radius,
    const Vec3& scale,
    const renderer::Material& material
)
{
    if (!_device || !_queue) return *this;

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    entity.mesh = renderer::Mesh::createSphere(_device, _queue, radius, 32, 32);
    entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.localBounds = renderer::BoundingBox{{-radius, -radius, -radius}, {radius, radius, radius}};

    _scene.addEntity(std::move(entity));
    return *this;
}

SceneBuilder& SceneBuilder::addPyramid(
    std::string name,
    const Vec3& position,
    float baseWidth,
    float height,
    const renderer::Material& material
)
{
    if (!_device || !_queue) return *this;

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    entity.mesh = renderer::Mesh::createPyramid(_device, _queue, baseWidth, height);
    entity.texture = renderer::Texture::createGridPattern(_device, _queue, 256, 256);
    entity.material = material;
    entity.position = position;
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
    const renderer::Material& material
)
{
    if (!_device || !_queue) return *this;

    renderer::MeshData data = renderer::MeshLoader::parseOBJFile(objFilePath);
    if (!data.success) {
        CORIUM_LOG_WARN("SceneBuilder", "Failed to load OBJ model from path: ", objFilePath);
        return *this;
    }

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    entity.mesh.upload(_device, _queue, data.vertices, data.indices);
    entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
    entity.rotation = rotation;
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
    const renderer::Material& material
)
{
    if (!_device || !_queue) return *this;

    SimEntity entity{};
    entity.id = _nextEntityId++;
    entity.name = std::move(name);
    entity.mesh = renderer::Mesh::createCylinder(_device, _queue, radius, height, segments);
    entity.texture = renderer::Texture::createCheckerboard(_device, _queue, 256, 256, 32);
    entity.material = material;
    entity.position = position;
    entity.scale = scale;
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
    joint.name = std::move(name);
    joint.parentName = std::move(parentName);
    joint.childName = std::move(childName);
    joint.type = type;
    joint.anchor = anchor;
    joint.axis = axis;
    joint.minLimit = minLimit;
    joint.maxLimit = maxLimit;

    _scene.addJoint(std::move(joint));
    return *this;
}

SimScene SceneBuilder::build()
{
    CORIUM_LOG_INFO("SceneBuilder", "Finalized 3D simulation environment scene with ",
                    _scene.entityCount(), " entities.");
    return std::move(_scene);
}

} // namespace corium_sim::scene
