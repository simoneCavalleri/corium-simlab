#pragma once

#include "corium_sim/SimConfig.hpp"
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/physics/PhysicsEngine.hpp"
#include "corium_sim/renderer/Camera.hpp"
#include "corium_sim/renderer/Mesh.hpp"
#include "corium_sim/renderer/SensorCamera.hpp"
#include "corium_sim/renderer/Texture.hpp"
#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/scene/SimScene.hpp"
#include "corium_sim/scene/SceneBuilder.hpp"
#include "corium_sim/environment/SimEnvironment.hpp"

#include <corium/corium.hpp>
#include <corium/ui/WindowApp.hpp>
#include <corium/ui/backends/GlfwWindowBackend.hpp>

#include "corium_sim/kinematics/JointKinematics.hpp"

namespace corium_sim {

// Runtime configuration using DefaultSimEvents
using SimRuntime = corium::RuntimeBuilder<>
    ::WithEvents<DefaultSimEvents>
    ::WithPriorityQueue<256, 2048>
    ::Build;

/// @brief Main Corium SimLab Application Class for Agent Simulation & 3D Visualization.
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

    /// @brief Set active 3D simulation scene environment.
    void setScene(scene::SimScene scene) noexcept;

    /// @brief Reset agent training environment state to initial pose & step count.
    void resetEnvironment() noexcept;

    /// @brief Public API to load 3D simulation asset mesh into active scene.
    bool loadSceneMesh(const std::string& filePath);

    [[nodiscard]] renderer::WebGpuBackend& renderer() noexcept { return _gpuBackend; }
    [[nodiscard]] const renderer::WebGpuBackend& renderer() const noexcept { return _gpuBackend; }
    [[nodiscard]] renderer::Camera& camera() noexcept { return _camera; }
    [[nodiscard]] renderer::SensorCamera& sensorCamera() noexcept { return _sensorCamera; }
    [[nodiscard]] environment::DefaultSimEnvironment& environment() noexcept { return _environment; }
    [[nodiscard]] const environment::DefaultSimEnvironment& environment() const noexcept { return _environment; }
    [[nodiscard]] scene::SimScene& scene() noexcept { return _environment.scene(); }
    [[nodiscard]] const scene::SimScene& scene() const noexcept { return _environment.scene(); }
    [[nodiscard]] physics::PhysicsEngine& physics() noexcept { return _environment.physics(); }
    [[nodiscard]] kinematics::JointKinematics& jointKinematics() noexcept { return _jointKinematics; }


    /// @brief Set simulation configuration parameters.
    void setConfig(const SimConfig& config) noexcept { _config = config; }
    [[nodiscard]] const SimConfig& config() const noexcept { return _config; }
    [[nodiscard]] SimConfig& config() noexcept { return _config; }

private:
    renderer::WebGpuBackend _gpuBackend{};
    renderer::Camera _camera{};
    renderer::SensorCamera _sensorCamera{128, 128, 75.0f};
    renderer::OffscreenTarget _sensorTarget{};

    environment::DefaultSimEnvironment _environment{};

    kinematics::JointKinematics _jointKinematics{};
    SimConfig _config{};


    uint32_t _renderFramesCount = 0;
    uint64_t _simStepCount = 0;
    uint64_t _episodeCount = 0;
};

} // namespace corium_sim
