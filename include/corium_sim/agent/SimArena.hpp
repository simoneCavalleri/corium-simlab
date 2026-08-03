#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "corium_sim/agent/AgentSpec.hpp"
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/agent/PhysicalAgent.hpp"
#include "corium_sim/environment/RewardBuilder.hpp"
#include "corium_sim/environment/SimEnvironment.hpp"
#include "corium_sim/math/Math.hpp"
#include "corium_sim/physics/PhysicsEngine.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent {

/// @brief Finalized SimArena Instance containing 3D Environment Scene, Physical Agent(s), Policy, and Reward/Task Engine.
template <
    physics::PhysicsBackend PhysicsEngineType,
    typename AgentSpecType
>
class SimArenaCompleted {
public:
    using AgentType = typename AgentSpecType::AgentType;

    SimArenaCompleted(
        environment::SimEnvironment<PhysicsEngineType> env,
        AgentType agent,
        typename AgentSpecType::Policy policy,
        environment::RewardBuilder rewardEngine = {}
    ) : _env(std::move(env)),
        _agent(std::move(agent)),
        _policy(std::move(policy)),
        _rewardEngine(std::move(rewardEngine)) {}

    [[nodiscard]] environment::SimEnvironment<PhysicsEngineType>& env() noexcept { return _env; }
    [[nodiscard]] const environment::SimEnvironment<PhysicsEngineType>& env() const noexcept { return _env; }
    [[nodiscard]] std::shared_ptr<scene::SimScene> scenePtr() const noexcept { return _env.scenePtr(); }

    [[nodiscard]] AgentType& agent() noexcept { return _agent; }
    [[nodiscard]] const AgentType& agent() const noexcept { return _agent; }

    [[nodiscard]] typename AgentSpecType::Policy& policy() noexcept { return _policy; }
    [[nodiscard]] const typename AgentSpecType::Policy& policy() const noexcept { return _policy; }

    [[nodiscard]] environment::RewardBuilder& rewardEngine() noexcept { return _rewardEngine; }
    [[nodiscard]] const environment::RewardBuilder& rewardEngine() const noexcept { return _rewardEngine; }

    /// @brief Set active WebGPU rendering backend for GPU-accelerated sensors (Cameras, GPU LiDAR).
    void setRenderBackend(renderer::WebGpuBackend* backend) noexcept { _renderBackend = backend; }
    [[nodiscard]] renderer::WebGpuBackend* renderBackend() const noexcept { return _renderBackend; }

    // -------------------------------------------------------------------------
    // Main control loop step
    // -------------------------------------------------------------------------

    /// @brief Execute a single high-frequency simulation step:
    ///        Perception → Policy → Action → Physics → Reward + Done check.
    /// @return TaskStepResult containing reward, done, truncated flags and info string.
    [[nodiscard]] environment::TaskStepResult step(float dt = 0.01667f) noexcept
    {
        // 1. Sense (forwarding optional _renderBackend for GPU sensors)
        auto obs = _agent.observe(_env.scene(), _renderBackend);

        // 2. Plan
        auto action = _policy.plan(obs);

        // 3. Actuate
        _agent.actuate(action);

        // 4. Write velocity/angular velocity into the shared scene entity
        if (auto* sceneEntity = _env.scene().findEntity(_agent.body().name)) {
            sceneEntity->velocity        = _agent.body().velocity;
            sceneEntity->angularVelocity = _agent.body().angularVelocity;
        }

        // 5. Physics step (authoritative)
        _env.step(dt);

        // 6. Read back integrated position/rotation from physics
        if (auto* sceneEntity = _env.scene().findEntity(_agent.body().name)) {
            _agent.body().position        = sceneEntity->position;
            _agent.body().rotation        = sceneEntity->rotation;
            _agent.body().velocity        = sceneEntity->velocity;
            _agent.body().angularVelocity = sceneEntity->angularVelocity;
        }

        // 7. Reward + termination via unified RewardBuilder / ITask
        std::span<const float> actionSpan(action.data(), action.size());
        _rewardEngine.updateContext(_agent.body().position, actionSpan, /*isCollided=*/false);

        return _rewardEngine.computeResult();
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
    AgentType _agent;
    typename AgentSpecType::Policy _policy;
    environment::RewardBuilder _rewardEngine;
    renderer::WebGpuBackend* _renderBackend = nullptr;
};

// =============================================================================
// Fluent Builder Stages
// =============================================================================

/// @brief Fluent Builder Stage 3: Attach Reward Policy & Finalize SimArena Assembly.
template <physics::PhysicsBackend PhysicsEngineType, typename AgentSpecType>
class SimArenaRewardStage {
public:
    SimArenaRewardStage(
        environment::SimEnvironment<PhysicsEngineType> env,
        typename AgentSpecType::AgentType agent,
        typename AgentSpecType::Policy policy
    ) : _env(std::move(env)), _agent(std::move(agent)), _policy(std::move(policy)) {}

    /// @brief Attach composable reward / task engine and finalize.
    auto withRewardPolicy(environment::RewardBuilder rewardEngine)
    {
        auto rewardPtr = std::make_shared<environment::RewardBuilder>(std::move(rewardEngine));
        _env.setTask(rewardPtr);
        return SimArenaCompleted<PhysicsEngineType, AgentSpecType>(
            std::move(_env), std::move(_agent), std::move(_policy), std::move(*rewardPtr)
        );
    }

    /// @brief Finalize SimArena environment without a specific reward policy.
    auto build()
    {
        return SimArenaCompleted<PhysicsEngineType, AgentSpecType>(
            std::move(_env), std::move(_agent), std::move(_policy)
        );
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
    typename AgentSpecType::AgentType _agent;
    typename AgentSpecType::Policy _policy;
};

/// @brief Fluent Builder Stage 2: Spawn Agent from AgentSpec into SimArena.
template <physics::PhysicsBackend PhysicsEngineType>
class SimArenaSpawnStage {
public:
    explicit SimArenaSpawnStage(environment::SimEnvironment<PhysicsEngineType> env)
        : _env(std::move(env)) {}

    /// @brief Spawn an agent instance into the environment scene using a decoupled AgentSpec.
    template <typename AgentSpecType>
    auto spawnAgent(const std::string& name, AgentSpecType spec, math::Vec3 initialPosition = {0.0f, 0.5f, 0.0f})
    {
        scene::SimEntity agentBody = std::move(spec.model());
        agentBody.name     = name;
        agentBody.position = initialPosition;

        scene::SimEntity sceneEntity{};
        sceneEntity.name        = name;
        sceneEntity.position    = initialPosition;
        sceneEntity.mass        = agentBody.mass;
        sceneEntity.scale       = agentBody.scale;
        sceneEntity.localBounds = agentBody.localBounds;

        _env.scene().addEntity(std::move(sceneEntity));

        using AgentType = typename AgentSpecType::AgentType;
        AgentType agent(std::move(agentBody), std::move(spec.perception()), std::move(spec.actuators()));

        return SimArenaRewardStage<PhysicsEngineType, AgentSpecType>(
            std::move(_env), std::move(agent), std::move(spec.policy())
        );
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
};

/// @brief Fluent Builder Stage 1: Configure Physics Backend & 3D Environment Scene for SimArena.
template <physics::PhysicsBackend PhysicsEngineType = physics::PhysicsEngine>
class SimArenaEnvStage {
public:
    SimArenaEnvStage() = default;

    /// @brief Construct 3D environment scene via explicit scene callback.
    template <typename SceneFn>
    auto withEnvironment(SceneFn&& sceneFn)
    {
        scene::SimScene scene{};
        sceneFn(scene);
        environment::SimEnvironment<PhysicsEngineType> env(std::move(scene));
        return SimArenaSpawnStage<PhysicsEngineType>{std::move(env)};
    }
};

/// @brief Entry point for constructing 3D Simulation Arenas via Decoupled AgentSpecs.
template <physics::PhysicsBackend PhysicsEngineType = physics::PhysicsEngine>
[[nodiscard]] constexpr auto makeArena()
{
    return SimArenaEnvStage<PhysicsEngineType>{};
}

/// @brief Explicit alias for makeArena().
template <physics::PhysicsBackend PhysicsEngineType = physics::PhysicsEngine>
[[nodiscard]] constexpr auto makeSimArena()
{
    return makeArena<PhysicsEngineType>();
}

} // namespace corium_sim::agent
