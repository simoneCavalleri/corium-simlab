#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "corium_sim/SimConfig.hpp"
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/kinematics/JointKinematics.hpp"

#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/SensorCamera.hpp"
#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/scene/SimScene.hpp"

#include <corium/corium.hpp>
#include <corium/Runtime.hpp>
#include <corium/ui/WindowApp.hpp>
#include <corium/ui/backends/GlfwWindowBackend.hpp>

namespace corium_sim {

using SimRuntime = corium::RuntimeT<DefaultSimEvents>;

/// @brief Generic 3D WebGPU Simulation Application Shell.
class SimLabApp : public corium::ui::WindowApp<SimLabApp, corium::ui::GlfwWindow, SimRuntime::EventBusType> {
public:
    using BaseApp = corium::ui::WindowApp<SimLabApp, corium::ui::GlfwWindow, SimRuntime::EventBusType>;
    using StepCallback = std::function<void(SimLabApp& app, float dt)>;

    SimLabApp();
    ~SimLabApp();

    void onSetContext(corium::AppCoreContextT<SimRuntime::EventBusType> ctx);
    void onConfigureServices(corium::ServiceRegistryT<8, DefaultSimEvents>& registry);
    void onRegisterHandlers();
    void onInitialize();
    void onRender(double deltaTime);
    void onShutdown();

    /// @brief Set active 3D simulation scene (shared_ptr or move value).
    void setScene(std::shared_ptr<scene::SimScene> scene) noexcept;
    void setScene(scene::SimScene scene) noexcept;

    /// @brief Reset simulation environment state.
    void resetEnvironment() noexcept;

    /// @brief Load 3D mesh model into active scene.
    bool loadSceneMesh(const std::string& filePath);

    /// @brief Register custom simulation step callback (e.g. Incubator step / Policy step loop).
    void onStep(StepCallback callback) { _stepCallback = std::move(callback); }

    [[nodiscard]] renderer::WebGpuBackend& renderer() noexcept { return _gpuBackend; }
    [[nodiscard]] const renderer::WebGpuBackend& renderer() const noexcept { return _gpuBackend; }
    [[nodiscard]] renderer::Camera& camera() noexcept { return _camera; }
    [[nodiscard]] const renderer::Camera& camera() const noexcept { return _camera; }
    [[nodiscard]] renderer::SensorCamera& sensorCamera() noexcept { return _sensorCamera; }
    [[nodiscard]] std::shared_ptr<scene::SimScene> scenePtr() const noexcept { return _scene; }
    [[nodiscard]] scene::SimScene& scene() noexcept { return *_scene; }
    [[nodiscard]] const scene::SimScene& scene() const noexcept { return *_scene; }
    [[nodiscard]] kinematics::JointKinematics& jointKinematics() noexcept { return _jointKinematics; }

    [[nodiscard]] SimConfig& config() noexcept { return _config; }
    [[nodiscard]] const SimConfig& config() const noexcept { return _config; }
    void setConfig(const SimConfig& config) noexcept { _config = config; }

private:
    renderer::WebGpuBackend _gpuBackend{};
    renderer::Camera _camera{};
    renderer::SensorCamera _sensorCamera{128, 128, 75.0f};
    std::shared_ptr<scene::SimScene> _scene;
    kinematics::JointKinematics _jointKinematics{};
    SimConfig _config{};
    StepCallback _stepCallback{};

    uint32_t _renderFramesCount = 0;
    uint64_t _simStepCount = 0;
};

} // namespace corium_sim
