#pragma once

#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/Texture.hpp"
#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/services/TelemetryService.hpp"

#include <corium/corium.hpp>
#include <corium/ui/WindowApp.hpp>
#include <corium/ui/backends/GlfwWindowBackend.hpp>

namespace corium_sim {

// Runtime configuration using DefaultSimEvents
using SimRuntime = corium::RuntimeBuilder<>
    ::WithEvents<DefaultSimEvents>
    ::WithPriorityQueue<256, 2048>
    ::Build;

/// @brief Main Corium SimLab Application Class.
class SimLabApp : public corium::ui::WindowApp<SimLabApp, corium::ui::GlfwWindow, SimRuntime::EventBusType> {
public:
    using BaseApp = corium::ui::WindowApp<SimLabApp, corium::ui::GlfwWindow, SimRuntime::EventBusType>;

    SimLabApp();
    ~SimLabApp();

    void onSetContext(corium::AppCoreContextT<SimRuntime::EventBusType> ctx);
    void onConfigureServices(corium::ServiceRegistryT<8, DefaultSimEvents>& registry);
    void onRegisterHandlers();
    void onInitialize();
    void onRender(double deltaTime);
    void onShutdown();

    [[nodiscard]] renderer::WebGpuBackend& renderer() noexcept { return _gpuBackend; }
    [[nodiscard]] const renderer::WebGpuBackend& renderer() const noexcept { return _gpuBackend; }
    [[nodiscard]] renderer::Camera& camera() noexcept { return _camera; }

private:
    renderer::WebGpuBackend _gpuBackend{};
    services::TelemetryService<DefaultSimEvents> _telemetryService{};

    renderer::Camera _camera{};
    renderer::Mesh _cubeMesh{};
    renderer::Mesh _sphereMesh{};
    renderer::Mesh _planeMesh{};
    renderer::Mesh _pyramidMesh{};

    renderer::Texture _checkerTex{};
    renderer::Texture _gridTex{};

    math::Vec3 _agentPosition{0.0f, 0.75f, 0.0f};
    float _totalTime = 0.0f;
    uint32_t _sensorEventCount = 0;
    uint32_t _poseEventCount = 0;
    uint32_t _renderFramesCount = 0;
};

} // namespace corium_sim
