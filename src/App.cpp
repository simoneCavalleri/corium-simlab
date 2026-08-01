#include "corium_sim/App.hpp"
#include <iostream>
#include <cmath>

namespace corium_sim {

using namespace math;

SimLabApp::SimLabApp()
    : BaseApp({ .title = "Corium SimLab — WebGPU 3D Simulation Engine", .width = 1280, .height = 720 })
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

    // Mouse Movement -> Camera Orbit Yaw/Pitch
    on([this](const corium::ui::MouseMoveEvent& evt) {
        _camera.onMouseMove(evt.x, evt.y);
    });

    // Mouse Button -> Camera Orbit Dragging
    on([this](const corium::ui::MouseButtonEvent& evt) {
        _camera.onMouseButton(evt.button, evt.pressed, evt.x, evt.y);
    });
}

void SimLabApp::onConfigureServices(corium::ServiceRegistryT<8, DefaultSimEvents>& registry)
{
    _telemetryService.setFrequency(std::chrono::milliseconds(20)); // 50 Hz telemetry stream
    registry.registerService(_telemetryService);
}

void SimLabApp::onRegisterHandlers()
{
    // Telemetry Sensor Event Handler
    on([this](const TelemetryDataEvent& evt) {
        _sensorEventCount++;
        if (_sensorEventCount % 50 == 0) {
            std::cout << "[SimLab] Telemetry Sensor #" << evt.sensorId
                      << " | Sample " << evt.sampleCount
                      << " | Value: " << evt.value << "\n";
        }
    });

    // Agent Pose Event Handler (Updates 3D Agent Position)
    on([this](const AgentPoseEvent& evt) {
        _poseEventCount++;
        _agentPosition = Vec3{ evt.posX, evt.posY + 0.75f, evt.posZ };
    });

    // Key press event handler (ESC key -> exit)
    on([this](const corium::ui::KeyEvent& evt) {
        if (evt.key == 256 && evt.pressed) { // GLFW_KEY_ESCAPE
            std::cout << "[SimLab] ESC key pressed, shutting down...\n";
            requestQuit();
        }
    });
}

void SimLabApp::onInitialize()
{
    WGPUDevice device = _gpuBackend.device();
    WGPUQueue queue = _gpuBackend.queue();

    if (device && queue) {
        // Build 3D Primitives
        _cubeMesh = renderer::Mesh::createCube(device, queue, 1.5f);
        _sphereMesh = renderer::Mesh::createSphere(device, queue, 1.0f, 32, 32);
        _planeMesh = renderer::Mesh::createPlane(device, queue, 30.0f, 30.0f, 30);
        _pyramidMesh = renderer::Mesh::createPyramid(device, queue, 1.2f, 1.8f);

        // Build Procedural Textures
        _checkerTex = renderer::Texture::createCheckerboard(device, queue, 256, 256, 32);
        _gridTex = renderer::Texture::createGridPattern(device, queue, 256, 256);
    }

    _camera.setTarget(Vec3{0.0f, 1.0f, 0.0f});
    _camera.setDistance(10.0f);

    std::cout << "[SimLab] 3D WebGPU Engine & Scene Geometry Initialized.\n";
}

void SimLabApp::onRender(double deltaTime)
{
    float dt = static_cast<float>(deltaTime);
    _totalTime = 0;
    _renderFramesCount++;

    if (!_gpuBackend.beginFrame(_camera, _totalTime)) {
        return;
    }

    // 1. Render Ground Grid Plane
    Mat4 planeModel = Mat4::identity();
    _gpuBackend.drawMesh(_planeMesh, _gridTex, planeModel);

    // 2. Render Rotating Textured 3D Cube
    Mat4 cubeModel = Mat4::translate(Vec3{-2.8f, 1.2f, 0.0f})
                   * Mat4::rotateY(_totalTime * 0.8f)
                   * Mat4::rotateX(_totalTime * 0.4f);
    _gpuBackend.drawMesh(_cubeMesh, _checkerTex, cubeModel);

    // 3. Render Animated Floating 3D UV Sphere
    float sphereHeight = 1.4f + 0.4f * std::sin(_totalTime * 2.5f);
    Mat4 sphereModel = Mat4::translate(Vec3{2.8f, sphereHeight, 0.0f})
                     * Mat4::rotateY(_totalTime * 1.2f);
    _gpuBackend.drawMesh(_sphereMesh, _checkerTex, sphereModel);

    // 4. Render Telemetry Simulation Agent Pyramid
    Mat4 agentModel = Mat4::translate(_agentPosition)
                    * Mat4::rotateY(_totalTime * 2.0f);
    _gpuBackend.drawMesh(_pyramidMesh, _gridTex, agentModel);

    _gpuBackend.endFrame();
}

void SimLabApp::onShutdown()
{
    _cubeMesh.destroy();
    _sphereMesh.destroy();
    _planeMesh.destroy();
    _pyramidMesh.destroy();
    _checkerTex.destroy();
    _gridTex.destroy();

    _gpuBackend.shutdown();
    std::cout << "[SimLab] Application Shutdown Summary:\n";
    std::cout << "  - Total Sensor Samples Processed: " << _sensorEventCount << "\n";
    std::cout << "  - Total Agent Poses Processed:   " << _poseEventCount << "\n";
    std::cout << "  - Total Render Frames Executed:  " << _renderFramesCount << "\n";
}

} // namespace corium_sim
