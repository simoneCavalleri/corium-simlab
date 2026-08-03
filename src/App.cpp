#include "corium_sim/App.hpp"
#include "corium_sim/Log.hpp"
#include "corium_sim/renderer/MeshLoader.hpp"

namespace corium_sim {

using namespace math;

SimLabApp::SimLabApp()
    : BaseApp({ .title = "Corium SimLab — Physical Agent Incubator & Simulation Engine", .width = 1280, .height = 720, .noApi = true }),
      _scene(std::make_shared<scene::SimScene>())
{}

SimLabApp::~SimLabApp() = default;

void SimLabApp::onSetContext(corium::AppCoreContextT<SimRuntime::EventBusType> ctx)
{
    BaseApp::onSetContext(ctx);

    // Initialize WebGPU context with GLFW native handle
    GLFWwindow* nativeWin = window().backend().nativeHandle();
    _gpuBackend.initialize(
        nativeWin,
        windowConfig().width,
        windowConfig().height
    );

    _camera.setViewportSize(windowConfig().width, windowConfig().height);

    // Automatically update WebGPU surface viewport on window resize
    on([this](const corium::ui::WindowResizeEvent& evt) {
        _gpuBackend.resize(static_cast<uint32_t>(evt.width), static_cast<uint32_t>(evt.height));
        _camera.setViewportSize(static_cast<uint32_t>(evt.width), static_cast<uint32_t>(evt.height));
    });

    // Mouse Movement -> Orbit / Pan Camera
    on([this](const corium::ui::MouseMoveEvent& evt) {
        _camera.onMouseMove(evt.x, evt.y);
    });

    // Mouse Button -> Drag Start/Stop
    on([this](const corium::ui::MouseButtonEvent& evt) {
        _camera.onMouseButton(evt.button, evt.pressed, evt.x, evt.y);
    });

    // Mouse Wheel Scroll -> Zoom
    on([this](const corium::ui::MouseScrollEvent& evt) {
        _camera.onScroll(evt.xoffset, evt.yoffset);
    });

    // Key Press / Release -> Navigation & ESC Quit
    on([this](const corium::ui::KeyEvent& evt) {
        _camera.onKey(evt.key, evt.pressed);

        if (evt.key == 256 && evt.pressed) { // GLFW_KEY_ESCAPE
            CORIUM_LOG_INFO("SimLab", "ESC key pressed, shutting down simulation environment...");
            requestQuit();
        }
    });
}

void SimLabApp::onConfigureServices(corium::ServiceRegistryT<8, DefaultSimEvents>& registry)
{
    (void)registry;
}

void SimLabApp::onRegisterHandlers()
{
}

void SimLabApp::onInitialize()
{
    _camera.setTarget(Vec3{0.0f, 0.5f, 0.0f});
    _camera.setDistance(12.0f);
    _camera.setViewportSize(windowConfig().width, windowConfig().height);

    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();

    if (device && queue && _scene) {
        for (auto& entity : _scene->entities()) {
            if (!entity.mesh.isValid()) {
                switch (entity.shape) {
                    case scene::EntityShape::PlaneGrid:
                        entity.mesh = renderer::Mesh::createPlane(device, queue, 60.0f, 60.0f, 60);
                        entity.texture = renderer::Texture::createGridPattern(device, queue, 512, 512);
                        if (entity.material.albedo.x == 0.8f && entity.material.albedo.y == 0.8f && entity.material.albedo.z == 0.8f) {
                            entity.material = renderer::Material::Matte({0.3f, 0.35f, 0.40f, 1.0f});
                        }
                        entity.localBounds = renderer::BoundingBox{{-30.0f, 0.0f, -30.0f}, {30.0f, 0.0f, 30.0f}};
                        break;
                    case scene::EntityShape::Sphere:
                        entity.mesh = renderer::Mesh::createSphere(device, queue, 0.5f, 16);
                        entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                        entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                        break;
                    case scene::EntityShape::Cylinder:
                        entity.mesh = renderer::Mesh::createCylinder(device, queue, 0.5f, 1.0f, 16);
                        entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                        entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                        break;
                    case scene::EntityShape::Cube:
                    default:
                        entity.mesh = renderer::Mesh::createCube(device, queue, 1.0f);
                        entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                        entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                        break;
                }
            }
        }
        if (_scene->entityCount() > 0) {
            renderer::BoundingBox bounds = _scene->sceneBounds();
            _camera.focusOnBounds(bounds.min, bounds.max);
        }
    }
}

void SimLabApp::setScene(std::shared_ptr<scene::SimScene> scene) noexcept
{
    if (scene) {
        _scene = std::move(scene);
    } else {
        _scene = std::make_shared<scene::SimScene>();
    }

    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();
    if (device && queue && _scene) {
        for (auto& entity : _scene->entities()) {
            if (!entity.mesh.isValid()) {
                switch (entity.shape) {
                    case scene::EntityShape::PlaneGrid:
                        entity.mesh = renderer::Mesh::createPlane(device, queue, 60.0f, 60.0f, 60);
                        entity.texture = renderer::Texture::createGridPattern(device, queue, 512, 512);
                        if (entity.material.albedo.x == 0.8f && entity.material.albedo.y == 0.8f && entity.material.albedo.z == 0.8f) {
                            entity.material = renderer::Material::Matte({0.3f, 0.35f, 0.40f, 1.0f});
                        }
                        entity.localBounds = renderer::BoundingBox{{-30.0f, 0.0f, -30.0f}, {30.0f, 0.0f, 30.0f}};
                        break;
                    case scene::EntityShape::Sphere:
                        entity.mesh = renderer::Mesh::createSphere(device, queue, 0.5f, 16);
                        entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                        entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                        break;
                    case scene::EntityShape::Cylinder:
                        entity.mesh = renderer::Mesh::createCylinder(device, queue, 0.5f, 1.0f, 16);
                        entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                        entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                        break;
                    case scene::EntityShape::Cube:
                    default:
                        entity.mesh = renderer::Mesh::createCube(device, queue, 1.0f);
                        entity.texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
                        entity.localBounds = renderer::BoundingBox{{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}};
                        break;
                }
            }
        }
        if (_scene->entityCount() > 0) {
            renderer::BoundingBox bounds = _scene->sceneBounds();
            _camera.focusOnBounds(bounds.min, bounds.max);
        }
    }
}

void SimLabApp::setScene(scene::SimScene scene) noexcept
{
    setScene(std::make_shared<scene::SimScene>(std::move(scene)));
}

void SimLabApp::resetEnvironment() noexcept
{
    _simStepCount = 0;
}

bool SimLabApp::loadSceneMesh(const std::string& filePath)
{
    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();
    if (!device || !queue || !_scene) return false;

    _scene->addEntity(
        scene::SimEntity{
            .id = static_cast<uint32_t>(_scene->entityCount() + 1),
            .name = filePath,
            .mesh = renderer::Mesh::createFromOBJ(device, queue, filePath),
            .texture = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32),
            .position = Vec3{0.0f, 0.0f, 0.0f}
        }
    );
    return true;
}

void SimLabApp::onRender(double deltaTime)
{
    float dt = static_cast<float>(deltaTime);
    _renderFramesCount++;

    if (_stepCallback) {
        _stepCallback(*this, dt);
    }
    if (_scene) {
        _jointKinematics.updateKinematics(*_scene, dt);
    }
    _simStepCount++;

    _camera.update(dt);

    if (_gpuBackend.beginFrame(_camera) && _scene) {
        for (const auto& entity : _scene->entities()) {
            if (entity.mesh.isValid()) {
                _gpuBackend.drawMesh(entity.mesh, entity.texture, entity.transformMatrix(), entity.material);
            }
        }
        _gpuBackend.endFrame();
    }
}

void SimLabApp::onShutdown()
{
    if (_scene) {
        _scene->destroy();
    }
    _gpuBackend.shutdown();

    CORIUM_LOG_INFO("SimLab", "Application Shutdown Summary:");
    CORIUM_LOG_INFO("SimLab", "  - Total Simulation Steps: ", _simStepCount);
    CORIUM_LOG_INFO("SimLab", "  - Total Render Frames:    ", _renderFramesCount);
}

} // namespace corium_sim
