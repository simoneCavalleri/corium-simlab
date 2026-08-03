#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>

#include "corium_sim/environment/SimEnvironment.hpp"
#include "corium_sim/environment/Task.hpp"
#include "corium_sim/physics/PhysicsConcept.hpp"
#include "corium_sim/physics/PhysicsEngine.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::environment {

/// @brief Multi-Agent Simulation Environment with a single shared 3D SimScene.
///
/// Manages N heterogeneous or homogeneous physical agents coexisting in **one**
/// shared SimScene, advancing physics once per step for the whole world.
/// Agents interact with each other through the shared scene (collisions, LiDAR
/// cross-detection, formation control, etc.).
///
/// @tparam PhysicsEngineType Pluggable physics engine backend.
/// @tparam AgentTypes Heterogeneous physical agent types.
template <physics::PhysicsBackend PhysicsEngineType, typename... AgentTypes>
class MultiAgentEnvironment {
public:
    static constexpr std::size_t agent_count = sizeof...(AgentTypes);

    MultiAgentEnvironment()
        : _scene(std::make_shared<scene::SimScene>()) {}

    explicit MultiAgentEnvironment(scene::SimScene scene, PhysicsEngineType physicsEngine = {})
        : _scene(std::make_shared<scene::SimScene>(std::move(scene)))
        , _physicsEngine(std::move(physicsEngine)) {}

    explicit MultiAgentEnvironment(std::shared_ptr<scene::SimScene> scene, PhysicsEngineType physicsEngine = {})
        : _scene(std::move(scene))
        , _physicsEngine(std::move(physicsEngine))
    {
        if (!_scene) {
            _scene = std::make_shared<scene::SimScene>();
        }
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------

    /// @brief Advance the whole multi-agent world by dt seconds (single physics tick).
    void step(float dt = 0.01667f) noexcept
    {
        if (_scene) {
            _physicsEngine.step(*_scene, dt);
        }
        _currentStep++;
        _elapsedTime += dt;
    }

    /// @brief Reset environment state.
    void reset() noexcept
    {
        _physicsEngine.reset();
        _currentStep  = 0;
        _elapsedTime  = 0.0f;
        if (_task) {
            _task->reset();
        }
    }

    // -------------------------------------------------------------------------
    // Task
    // -------------------------------------------------------------------------

    /// @brief Attach an optional task definition for reward and termination.
    void setTask(std::shared_ptr<ITask> task) noexcept
    {
        _task = std::move(task);
    }

    /// @brief Evaluate the current task step result (reward, done, truncated).
    [[nodiscard]] TaskStepResult evaluateTask() noexcept
    {
        if (_task) {
            return _task->computeResult();
        }
        return TaskStepResult{};
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /// @brief Shared scene pointer — pass to SimLabApp::setScene() for rendering.
    [[nodiscard]] std::shared_ptr<scene::SimScene> scenePtr() const noexcept { return _scene; }

    [[nodiscard]] scene::SimScene& scene() noexcept         { return *_scene; }
    [[nodiscard]] const scene::SimScene& scene() const noexcept { return *_scene; }

    [[nodiscard]] PhysicsEngineType& physics() noexcept         { return _physicsEngine; }
    [[nodiscard]] const PhysicsEngineType& physics() const noexcept { return _physicsEngine; }

    [[nodiscard]] uint64_t currentStep()  const noexcept { return _currentStep; }
    [[nodiscard]] float    elapsedTime()  const noexcept { return _elapsedTime; }

private:
    std::shared_ptr<scene::SimScene> _scene;
    PhysicsEngineType                _physicsEngine{};
    std::shared_ptr<ITask>           _task{nullptr};

    uint64_t _currentStep = 0;
    float    _elapsedTime = 0.0f;
};

/// @brief Alias for default rigid-body multi-agent environment.
template <typename... AgentTypes>
using DefaultMultiAgentEnvironment = MultiAgentEnvironment<physics::PhysicsEngine, AgentTypes...>;

} // namespace corium_sim::environment
