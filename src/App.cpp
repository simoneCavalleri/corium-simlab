#include "corium_sim/App.hpp"
#include <iostream>

namespace corium_sim {

SimLabApp::SimLabApp()
    : BaseApp({ .title = "Corium SimLab — Real-Time Simulation Engine", .width = 1280, .height = 720 })
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

    // Automatically update WebGPU surface viewport on window resize
    on([this](const corium::ui::WindowResizeEvent& evt) {
        _gpuBackend.resize(static_cast<uint32_t>(evt.width), static_cast<uint32_t>(evt.height));
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
        std::cout << "[SimLab] Sensor #" << evt.sensorId
                  << " | Sample " << evt.sampleCount
                  << " | Value: " << evt.value << "\n";
    });

    // Agent Pose Event Handler
    on([this](const AgentPoseEvent& evt) {
        _poseEventCount++;
        double r = 0.08 + static_cast<double>(evt.posX) * 0.05;
        _gpuBackend.setClearColor(r, 0.12, 0.20, 1.0);
        std::cout << "[SimLab] Agent #" << evt.agentId
                  << " Position: (" << evt.posX << ", " << evt.posY << ", " << evt.posZ << ")\n";
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
    std::cout << "[SimLab] Application & WebGPU Graphics Environment Initialized.\n";
}

void SimLabApp::onRender(double deltaTime)
{
    _gpuBackend.renderFrame(deltaTime);
    _renderFramesCount++;
}

void SimLabApp::onShutdown()
{
    _gpuBackend.shutdown();
    std::cout << "[SimLab] Application Shutdown Summary:\n";
    std::cout << "  - Total Sensor Samples Processed: " << _sensorEventCount << "\n";
    std::cout << "  - Total Agent Poses Processed:   " << _poseEventCount << "\n";
    std::cout << "  - Total Render Frames Executed:  " << _renderFramesCount << "\n";
}

} // namespace corium_sim
