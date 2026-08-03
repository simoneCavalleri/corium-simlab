#pragma once

#include <corium/IEventSink.hpp>
#include "corium_sim/agent/AgentSpec.hpp"
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/agent/PhysicalAgent.hpp"
#include "corium_sim/environment/RewardBuilder.hpp"
#include "corium_sim/environment/SimEnvironment.hpp"
#include "corium_sim/events/SimEvents.hpp"
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
        std::shared_ptr<AgentType> agentPtr,
        typename AgentSpecType::Policy policy,
        environment::RewardBuilder rewardEngine = {}
    ) : _env(std::move(env)),
        _agentPtr(std::move(agentPtr)),
        _policy(std::move(policy)),
        _rewardEngine(std::move(rewardEngine)) {}

    void setAgentApp(std::shared_ptr<app::ConfigurableSimAgentApp> agentApp, std::shared_ptr<runtime::AgentRuntime> agentRuntime)
    {
        _agentApp = std::move(agentApp);
        _agentRuntime = std::move(agentRuntime);
    }

    [[nodiscard]] std::shared_ptr<app::ConfigurableSimAgentApp> agentApp() const noexcept { return _agentApp; }
    [[nodiscard]] std::shared_ptr<runtime::AgentRuntime> agentRuntime() const noexcept { return _agentRuntime; }

    [[nodiscard]] environment::SimEnvironment<PhysicsEngineType>& env() noexcept { return _env; }
    [[nodiscard]] const environment::SimEnvironment<PhysicsEngineType>& env() const noexcept { return _env; }
    [[nodiscard]] std::shared_ptr<scene::SimScene> scenePtr() const noexcept { return _env.scenePtr(); }

    [[nodiscard]] AgentType& agent() noexcept { return *_agentPtr; }
    [[nodiscard]] const AgentType& agent() const noexcept { return *_agentPtr; }
    [[nodiscard]] std::shared_ptr<AgentType> agentPtr() const noexcept { return _agentPtr; }

    [[nodiscard]] typename AgentSpecType::Policy& policy() noexcept { return _policy; }
    [[nodiscard]] const typename AgentSpecType::Policy& policy() const noexcept { return _policy; }

    [[nodiscard]] environment::RewardBuilder& rewardEngine() noexcept { return _rewardEngine; }
    [[nodiscard]] const environment::RewardBuilder& rewardEngine() const noexcept { return _rewardEngine; }

    /// @brief Set active WebGPU rendering backend for GPU-accelerated sensors (Cameras, GPU LiDAR).
    void setRenderBackend(renderer::WebGpuBackend* backend) noexcept { _renderBackend = backend; }
    [[nodiscard]] renderer::WebGpuBackend* renderBackend() const noexcept { return _renderBackend; }

    /// @brief Attach Corium event sink handle for event-driven telemetry and tracing.
    void setEventSink(corium::IEventSinkT<DefaultSimEvents> sink) noexcept { _eventSink = sink; }
    [[nodiscard]] corium::IEventSinkT<DefaultSimEvents> eventSink() const noexcept { return _eventSink; }

    // -------------------------------------------------------------------------
    // Main control loop step
    // -------------------------------------------------------------------------

    /// @brief Execute a single high-frequency simulation step:
    ///        Perception → Policy → Action → Physics → Reward + Done check.
    /// @return TaskStepResult containing reward, done, truncated flags and info string.
    [[nodiscard]] environment::TaskStepResult step(float dt = 0.01667f) noexcept
    {
        _stepIndex++;

        // 1. Agent Step Execution (Coordinated via Corium SimAgentApp & BackgroundServices)
        _agentApp->step(_stepIndex, dt);

        // Write velocity/angular velocity into the shared scene entity
        if (auto* sceneEntity = _env.scene().findEntity(_agentPtr->body().name)) {
            sceneEntity->velocity        = _agentPtr->body().velocity;
            sceneEntity->angularVelocity = _agentPtr->body().angularVelocity;
        }

        // Physics step (authoritative)
        _env.step(dt);

        // Read back integrated position/rotation from physics
        if (auto* sceneEntity = _env.scene().findEntity(_agentPtr->body().name)) {
            _agentPtr->body().position        = sceneEntity->position;
            _agentPtr->body().rotation        = sceneEntity->rotation;
            _agentPtr->body().velocity        = sceneEntity->velocity;
            _agentPtr->body().angularVelocity = sceneEntity->angularVelocity;
        }

        // Reward + termination via unified RewardBuilder / ITask
        std::vector<float> defaultActionVec{0.0f, 0.0f};
        _rewardEngine.updateContext(_agentPtr->body().position, defaultActionVec, /*isCollided=*/false);
        auto result = _rewardEngine.computeResult();

        if (_eventSink) {
            _eventSink.post(AgentObservationEvent{
                .agentId = _agentPtr->body().id,
                .posX = _agentPtr->body().position.x,
                .posY = _agentPtr->body().position.y,
                .posZ = _agentPtr->body().position.z,
                .velX = _agentPtr->body().velocity.x,
                .velY = _agentPtr->body().velocity.y,
                .velZ = _agentPtr->body().velocity.z,
                .targetX = _rewardEngine.target().x,
                .targetY = _rewardEngine.target().y,
                .targetZ = _rewardEngine.target().z,
                .distanceToTarget = (_agentPtr->body().position - _rewardEngine.target()).length(),
                .reward = result.reward,
                .isTerminated = result.done,
                .isTruncated = result.truncated,
                .stepIndex = _stepIndex
            });
        }

        return result;
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
    std::shared_ptr<AgentType> _agentPtr;
    typename AgentSpecType::Policy _policy;
    environment::RewardBuilder _rewardEngine;
    renderer::WebGpuBackend* _renderBackend = nullptr;
    corium::IEventSinkT<DefaultSimEvents> _eventSink{};
    uint64_t _stepIndex = 0;
    std::shared_ptr<app::ConfigurableSimAgentApp> _agentApp{};
    std::shared_ptr<runtime::AgentRuntime> _agentRuntime{};
};

// =============================================================================
// Fluent Builder Stages
// =============================================================================

/// @brief Fluent Builder Stage 3: Attach Reward Policy & Finalize SimArena Assembly.
template <physics::PhysicsBackend PhysicsEngineType, typename AgentSpecType>
class SimArenaRewardStage {
public:
    using AgentType = typename AgentSpecType::AgentType;

    SimArenaRewardStage(
        environment::SimEnvironment<PhysicsEngineType> env,
        std::shared_ptr<AgentType> agentPtr,
        typename AgentSpecType::Policy policy
    ) : _env(std::move(env)), _agentPtr(std::move(agentPtr)), _policy(std::move(policy)) {}

    void setAgentApp(std::shared_ptr<app::ConfigurableSimAgentApp> agentApp, std::shared_ptr<runtime::AgentRuntime> agentRuntime)
    {
        _agentApp = std::move(agentApp);
        _agentRuntime = std::move(agentRuntime);
    }

    /// @brief Attach composable reward / task engine and finalize.
    auto withRewardPolicy(environment::RewardBuilder rewardEngine)
    {
        auto rewardPtr = std::make_shared<environment::RewardBuilder>(std::move(rewardEngine));
        _env.setTask(rewardPtr);
        auto arena = SimArenaCompleted<PhysicsEngineType, AgentSpecType>(
            std::move(_env), _agentPtr, std::move(_policy), std::move(*rewardPtr)
        );
        arena.setAgentApp(_agentApp, _agentRuntime);
        return arena;
    }

    /// @brief Finalize SimArena environment without a specific reward policy.
    auto build()
    {
        auto arena = SimArenaCompleted<PhysicsEngineType, AgentSpecType>(
            std::move(_env), _agentPtr, std::move(_policy)
        );
        arena.setAgentApp(_agentApp, _agentRuntime);
        return arena;
    }

private:
    environment::SimEnvironment<PhysicsEngineType> _env;
    std::shared_ptr<AgentType> _agentPtr;
    typename AgentSpecType::Policy _policy;
    std::shared_ptr<app::ConfigurableSimAgentApp> _agentApp{};
    std::shared_ptr<runtime::AgentRuntime> _agentRuntime{};
};

/// @brief Fluent Builder Stage 2: Spawn Agent from AgentSpec into SimArena.
template <physics::PhysicsBackend PhysicsEngineType>
class SimArenaSpawnStage {
public:
    explicit SimArenaSpawnStage(environment::SimEnvironment<PhysicsEngineType> env)
        : _env(std::move(env)) {}

    /// @brief Spawn an agent instance into the environment scene using a decoupled AgentSpec.
    /// Automatically instantiates SimAgentApp and initializes AgentRuntime behind the scenes!
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
        auto agentPtr = std::make_shared<AgentType>(
            std::move(agentBody),
            std::move(spec.perception()),
            std::move(spec.actuators())
        );

        // Automatically generate Corium AgentApp & initialize AgentRuntime behind the scenes!
        auto agentApp = spec.createApp(agentPtr, &_env.scene(), nullptr);
        auto agentRuntime = std::make_shared<runtime::AgentRuntime>();
        agentRuntime->initialize(*agentApp);

        auto rewardStage = SimArenaRewardStage<PhysicsEngineType, AgentSpecType>(
            std::move(_env), agentPtr, std::move(spec.policy())
        );
        rewardStage.setAgentApp(agentApp, agentRuntime);
        return rewardStage;
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
