#pragma once

#include <cstdint>
#include <tuple>
#include <utility>

#include "corium_sim/environment/DomainRandomizer.hpp"
#include "corium_sim/environment/Task.hpp"
#include "corium_sim/physics/PhysicsConcept.hpp"
#include "corium_sim/physics/PhysicsEngine.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::environment {

/// @brief Multi-Agent Simulation Environment container.
/// Manages a shared 3D SimScene containing multiple physical agents (heterogeneous or homogeneous)
/// stepping physics, perception, and actions concurrently.
/// @tparam PhysicsEngineType Pluggable physics engine backend.
/// @tparam AgentTypes Heterogeneous physical agent types.
template <physics::PhysicsBackend PhysicsEngineType, typename... AgentTypes>
class MultiAgentEnvironment {
public:
    static constexpr std::size_t agent_count = sizeof...(AgentTypes);

    MultiAgentEnvironment() = default;

    explicit MultiAgentEnvironment(scene::SimScene scene, PhysicsEngineType physicsEngine = {})
        : _scene(std::move(scene)), _physicsEngine(std::move(physicsEngine)) {}

    /// @brief Advance multi-agent physical simulation step by dt seconds.
    void step(float dt = 0.01667f) noexcept
    {
        _physicsEngine.step(_scene, dt);
        _currentStep++;
        _elapsedTime += dt;
    }

    /// @brief Reset environment state and apply domain randomization if enabled.
    void reset() noexcept
    {
        _physicsEngine.reset();
        _domainRandomizer.randomizeScene(_scene);
        _currentStep = 0;
        _elapsedTime = 0.0f;
    }

    [[nodiscard]] scene::SimScene& scene() noexcept { return _scene; }
    [[nodiscard]] const scene::SimScene& scene() const noexcept { return _scene; }

    [[nodiscard]] PhysicsEngineType& physics() noexcept { return _physicsEngine; }
    [[nodiscard]] const PhysicsEngineType& physics() const noexcept { return _physicsEngine; }

    [[nodiscard]] DomainRandomizer& randomizer() noexcept { return _domainRandomizer; }
    [[nodiscard]] const DomainRandomizer& randomizer() const noexcept { return _domainRandomizer; }

    [[nodiscard]] uint64_t currentStep() const noexcept { return _currentStep; }
    [[nodiscard]] float elapsedTime() const noexcept { return _elapsedTime; }

private:
    scene::SimScene _scene{};
    PhysicsEngineType _physicsEngine{};
    DomainRandomizer _domainRandomizer{};

    uint64_t _currentStep = 0;
    float _elapsedTime = 0.0f;
};

} // namespace corium_sim::environment
