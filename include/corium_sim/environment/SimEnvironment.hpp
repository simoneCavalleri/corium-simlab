#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include "corium_sim/environment/Task.hpp"
#include "corium_sim/physics/PhysicsConcept.hpp"
#include "corium_sim/physics/PhysicsEngine.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::environment {

/// @brief 3D Physical Agent Simulation Environment.
/// Manages the physical world scene, step dynamics, pluggable physics engine updates, and task reward evaluation.
/// @tparam PhysicsEngineType Pluggable physics engine backend satisfying physics::PhysicsBackend (default: physics::PhysicsEngine).
template <physics::PhysicsBackend PhysicsEngineType = physics::PhysicsEngine>
class SimEnvironment {
public:
    SimEnvironment()
        : _scene(std::make_shared<scene::SimScene>()) {}

    explicit SimEnvironment(scene::SimScene scene, PhysicsEngineType physicsEngine = {})
        : _scene(std::make_shared<scene::SimScene>(std::move(scene))), _physicsEngine(std::move(physicsEngine)) {}

    explicit SimEnvironment(std::shared_ptr<scene::SimScene> scene, PhysicsEngineType physicsEngine = {})
        : _scene(std::move(scene)), _physicsEngine(std::move(physicsEngine)) {
        if (!_scene) {
            _scene = std::make_shared<scene::SimScene>();
        }
    }

    /// @brief Advance the physical simulation by dt seconds using pluggable physics engine.
    void step(float dt = 0.01667f) noexcept
    {
        if (_scene) {
            _physicsEngine.step(*_scene, dt);
        }
        _currentStep++;
        _elapsedTime += dt;
    }

    /// @brief Reset environment state, entity positions, step count, and physics engine.
    void reset() noexcept
    {
        _physicsEngine.reset();
        _currentStep = 0;
        _elapsedTime = 0.0f;
        if (_task) {
            _task->reset();
        }
    }

    /// @brief Set active task definition for reward and termination evaluation.
    void setTask(std::shared_ptr<ITask> task) noexcept
    {
        _task = std::move(task);
    }

    /// @brief Compute step reward and completion status if a task is set.
    [[nodiscard]] TaskStepResult evaluateTask() noexcept
    {
        if (_task) {
            return _task->computeResult();
        }
        return TaskStepResult{};
    }

    [[nodiscard]] std::shared_ptr<scene::SimScene> scenePtr() const noexcept { return _scene; }
    [[nodiscard]] scene::SimScene& scene() noexcept { return *_scene; }
    [[nodiscard]] const scene::SimScene& scene() const noexcept { return *_scene; }

    [[nodiscard]] PhysicsEngineType& physics() noexcept { return _physicsEngine; }
    [[nodiscard]] const PhysicsEngineType& physics() const noexcept { return _physicsEngine; }

    [[nodiscard]] uint64_t currentStep() const noexcept { return _currentStep; }
    [[nodiscard]] float elapsedTime() const noexcept { return _elapsedTime; }

private:
    std::shared_ptr<scene::SimScene> _scene;
    PhysicsEngineType _physicsEngine{};
    std::shared_ptr<ITask> _task{nullptr};

    uint64_t _currentStep = 0;
    float _elapsedTime = 0.0f;
};

/// Alias for default rigid body simulation environment
using DefaultSimEnvironment = SimEnvironment<physics::PhysicsEngine>;

} // namespace corium_sim::environment
