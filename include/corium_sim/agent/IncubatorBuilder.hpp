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

/// @brief Finalized Incubator Instance containing Environment, Physical Agent(s), Policy, and Reward System.
template <
    physics::PhysicsBackend PhysicsEngineType,
    typename AgentSpecType
>
class IncubatorCompleted {
public:
    using AgentType = typename AgentSpecType::AgentType;

    IncubatorCompleted(
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

    [[nodiscard]] AgentType& agent() noexcept { return _agent; }
    [[nodiscard]] const AgentType& agent() const noexcept { return _agent; }

    [[nodiscard]] typename AgentSpecType::Policy& policy() noexcept { return _policy; }
    [[nodiscard]] const typename AgentSpecType::Policy& policy() const noexcept { return _policy; }

    [[nodiscard]] environment::RewardBuilder& rewardEngine() noexcept { return _rewardEngine; }
    [[nodiscard]] const environment::RewardBuilder& rewardEngine() const noexcept { return _rewardEngine; }

    /// @brief Execute a single high-frequency control step: Perception -> Policy -> Action -> Physics -> Reward.
    /// @return Scalar reward computed from reward policy.
    float step(float dt = 0.01667f) noexcept
    {
        auto obs = _agent.observe(_env.scene());
        auto action = _policy.plan(obs);
        _agent.actuate(action);
        _env.step(dt);

        math::Vec3 targetPos{4.0f, 0.75f, -2.0f};
        return _rewardEngine.computeTotalReward(_agent.body().position, targetPos, action);
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
    AgentType _agent;
    typename AgentSpecType::Policy _policy;
    environment::RewardBuilder _rewardEngine;
};

/// @brief Fluent Builder Stage 3: Attach Reward Policy & Finalize Incubator Assembly.
template <physics::PhysicsBackend PhysicsEngineType, typename AgentSpecType>
class IncubatorRewardStage {
public:
    IncubatorRewardStage(
        environment::SimEnvironment<PhysicsEngineType> env,
        typename AgentSpecType::AgentType agent,
        typename AgentSpecType::Policy policy
    ) : _env(std::move(env)), _agent(std::move(agent)), _policy(std::move(policy)) {}

    /// @brief Attach composable reward policy engine.
    auto withRewardPolicy(environment::RewardBuilder rewardEngine)
    {
        return IncubatorCompleted<PhysicsEngineType, AgentSpecType>(
            std::move(_env), std::move(_agent), std::move(_policy), std::move(rewardEngine)
        );
    }

    /// @brief Finalize incubator without specific reward terms.
    auto build()
    {
        return IncubatorCompleted<PhysicsEngineType, AgentSpecType>(
            std::move(_env), std::move(_agent), std::move(_policy)
        );
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
    typename AgentSpecType::AgentType _agent;
    typename AgentSpecType::Policy _policy;
};

/// @brief Fluent Builder Stage 2: Spawn Agent from AgentSpec.
template <physics::PhysicsBackend PhysicsEngineType>
class IncubatorSpawnStage {
public:
    explicit IncubatorSpawnStage(environment::SimEnvironment<PhysicsEngineType> env)
        : _env(std::move(env)) {}

    /// @brief Spawn an agent instance into the environment scene using a decoupled AgentSpec.
    template <typename AgentSpecType>
    auto spawnAgent(const std::string& name, AgentSpecType spec, math::Vec3 initialPosition = {0.0f, 0.5f, 0.0f})
    {
        scene::SimEntity agentBody = std::move(spec.model());
        agentBody.name = name;
        agentBody.position = initialPosition;

        scene::SimEntity sceneEntity{};
        sceneEntity.name = name;
        sceneEntity.position = initialPosition;
        sceneEntity.mass = agentBody.mass;
        sceneEntity.scale = agentBody.scale;
        sceneEntity.localBounds = agentBody.localBounds;

        _env.scene().addEntity(std::move(sceneEntity));

        using AgentType = typename AgentSpecType::AgentType;
        AgentType agent(std::move(agentBody), std::move(spec.perception()), std::move(spec.actuators()));

        return IncubatorRewardStage<PhysicsEngineType, AgentSpecType>(
            std::move(_env), std::move(agent), std::move(spec.policy())
        );
    }


private:
    environment::SimEnvironment<PhysicsEngineType> _env;
};

/// @brief Fluent Builder Stage 1: Configure Pluggable Physics & 3D Environment Scene.
template <physics::PhysicsBackend PhysicsEngineType = physics::PhysicsEngine>
class IncubatorEnvStage {
public:
    IncubatorEnvStage() = default;

    /// @brief Construct 3D environment scene via explicit scene callback.
    template <typename SceneFn>
    auto withEnvironment(SceneFn&& sceneFn)
    {
        scene::SimScene scene{};
        sceneFn(scene);
        environment::SimEnvironment<PhysicsEngineType> env(std::move(scene));
        return IncubatorSpawnStage<PhysicsEngineType>{std::move(env)};
    }
};

/// @brief Entry point for constructing Incubators via Decoupled AgentSpecs.
template <physics::PhysicsBackend PhysicsEngineType = physics::PhysicsEngine>
[[nodiscard]] constexpr auto makeIncubator()
{
    return IncubatorEnvStage<PhysicsEngineType>{};
}

} // namespace corium_sim::agent
