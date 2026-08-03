#pragma once

#include <cstddef>
#include <utility>

#include "corium_sim/agent/ActuatorSuite.hpp"
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/agent/PerceptionPipeline.hpp"
#include "corium_sim/agent/PhysicalAgent.hpp"
#include "corium_sim/agent/Policy.hpp"
#include "corium_sim/agent/SensorSuite.hpp"
#include "corium_sim/scene/SimEntity.hpp"

#include "corium_sim/app/SimAgentApp.hpp"

namespace corium_sim::agent {

// Forward Declarations for AgentSpec Stages
template <typename ModelType, typename SensorSuiteType>
class AgentSpecPerceptionStage;

template <typename ModelType, typename PerceptionType>
class AgentSpecActuatorsStage;

template <typename ModelType, typename PerceptionType, typename ActuatorSuiteType>
class AgentSpecPolicyStage;

/// @brief Finalized Agent Specification blueprint containing Model, Perception, Actuators, and Policy Brain.
template <typename ModelType, typename PerceptionType, typename ActuatorSuiteType, typename PolicyType>
class AgentSpec {
public:
    using Model = ModelType;
    using Perception = PerceptionType;
    using Actuators = ActuatorSuiteType;
    using Policy = PolicyType;
    using AgentType = PhysicalAgent<ModelType, PerceptionType, ActuatorSuiteType>;

    AgentSpec(ModelType model, PerceptionType perception, ActuatorSuiteType actuators, PolicyType policy)
        : _model(std::move(model)), _perception(std::move(perception)), _actuators(std::move(actuators)), _policy(std::move(policy)) {}

    AgentSpec(const AgentSpec&) = delete;
    AgentSpec& operator=(const AgentSpec&) = delete;

    AgentSpec(AgentSpec&&) noexcept = default;
    AgentSpec& operator=(AgentSpec&&) noexcept = default;

    [[nodiscard]] ModelType& model() noexcept { return _model; }
    [[nodiscard]] const ModelType& model() const noexcept { return _model; }

    [[nodiscard]] PerceptionType& perception() noexcept { return _perception; }
    [[nodiscard]] const PerceptionType& perception() const noexcept { return _perception; }

    [[nodiscard]] ActuatorSuiteType& actuators() noexcept { return _actuators; }
    [[nodiscard]] const ActuatorSuiteType& actuators() const noexcept { return _actuators; }

    [[nodiscard]] PolicyType& policy() noexcept { return _policy; }
    [[nodiscard]] const PolicyType& policy() const noexcept { return _policy; }

    /// @brief Automatically instantiate the underlying Corium SimAgentApp behind the scenes!
    /// Wires SensorService, PlannerService, and ActuatorService directly to target Agent instance.
    template <typename AgentType, typename SceneType = scene::SimScene>
    auto createApp(std::shared_ptr<AgentType> agentPtr, SceneType* scenePtr = nullptr, renderer::WebGpuBackend* renderBackend = nullptr)
    {
        auto agentApp = std::make_shared<app::ConfigurableSimAgentApp>();

        // Wire SensorService behind the scenes
        agentApp->sensorService().setSampleFunc([agentPtr, scenePtr, renderBackend](uint64_t stepIndex) -> std::vector<float> {
            if (!agentPtr) return {};
            if (!scenePtr) {
                scene::SimScene dummyScene;
                auto obs = agentPtr->observe(dummyScene, renderBackend, {}, stepIndex);
                return std::vector<float>(obs.begin(), obs.end());
            }
            auto obs = agentPtr->observe(*scenePtr, renderBackend, {}, stepIndex);
            return std::vector<float>(obs.begin(), obs.end());
        });

        // Wire PlannerService behind the scenes
        agentApp->plannerService().setPlanFunc([this](const std::vector<float>& obs) -> std::vector<float> {
            if constexpr (requires { _policy.plan(obs); }) {
                auto action = _policy.plan(obs);
                return std::vector<float>(action.begin(), action.end());
            } else {
                auto action = _policy(obs);
                return std::vector<float>(action.begin(), action.end());
            }
        });

        // Wire ActuatorService behind the scenes
        agentApp->actuatorService().setApplyFunc([agentPtr](std::span<const float> actions, uint64_t stepIndex) {
            if (agentPtr) {
                agentPtr->actuate(actions, {}, stepIndex);
            }
        });

        return agentApp;
    }

    /// @brief Standalone createApp overload using embedded model/perception/actuators.
    template <typename SceneType = scene::SimScene>
    auto createApp(SceneType* scenePtr = nullptr, renderer::WebGpuBackend* renderBackend = nullptr)
    {
        auto agentApp = std::make_shared<app::ConfigurableSimAgentApp>();

        agentApp->sensorService().setSampleFunc([this, scenePtr, renderBackend](uint64_t stepIndex) -> std::vector<float> {
            if (!scenePtr) {
                scene::SimScene dummyScene;
                auto obs = _perception.observe(_model, dummyScene, renderBackend, {}, stepIndex);
                return std::vector<float>(obs.begin(), obs.end());
            }
            auto obs = _perception.observe(_model, *scenePtr, renderBackend, {}, stepIndex);
            return std::vector<float>(obs.begin(), obs.end());
        });

        agentApp->plannerService().setPlanFunc([this](const std::vector<float>& obs) -> std::vector<float> {
            if constexpr (requires { _policy.plan(obs); }) {
                auto action = _policy.plan(obs);
                return std::vector<float>(action.begin(), action.end());
            } else {
                auto action = _policy(obs);
                return std::vector<float>(action.begin(), action.end());
            }
        });

        agentApp->actuatorService().setApplyFunc([this](std::span<const float> actions, uint64_t stepIndex) {
            _actuators.apply(_model, actions, {}, stepIndex);
        });

        return agentApp;
    }

private:
    ModelType _model;
    PerceptionType _perception;
    ActuatorSuiteType _actuators;
    PolicyType _policy;
};

/// @brief Fluent AgentSpec Stage 4: Attach Policy Brain / Planner.
template <typename ModelType, typename PerceptionType, typename ActuatorSuiteType>
class AgentSpecPolicyStage {
public:
    AgentSpecPolicyStage(ModelType model, PerceptionType perception, ActuatorSuiteType actuators)
        : _model(std::move(model)), _perception(std::move(perception)), _actuators(std::move(actuators)) {}

    /// @brief Attach planning policy / brain algorithm.
    template <typename PolicyType>
    auto withPolicy(PolicyType policy)
    {
        return AgentSpec<ModelType, PerceptionType, ActuatorSuiteType, PolicyType>(
            std::move(_model), std::move(_perception), std::move(_actuators), std::move(policy)
        );
    }

private:
    ModelType _model;
    PerceptionType _perception;
    ActuatorSuiteType _actuators;
};

/// @brief Fluent AgentSpec Stage 3: Attach Physical Actuators Suite.
template <typename ModelType, typename PerceptionType>
class AgentSpecActuatorsStage {
public:
    AgentSpecActuatorsStage(ModelType model, PerceptionType perception)
        : _model(std::move(model)), _perception(std::move(perception)) {}

    /// @brief Attach physical motor drives / actuators suite.
    template <concepts::Actuator... Actuators>
    auto withActuators(Actuators... actuators)
    {
        ActuatorSuite<Actuators...> actuatorSuite(std::move(actuators)...);
        return AgentSpecPolicyStage<ModelType, PerceptionType, ActuatorSuite<Actuators...>>(
            std::move(_model), std::move(_perception), std::move(actuatorSuite)
        );
    }

private:
    ModelType _model;
    PerceptionType _perception;
};

/// @brief Fluent AgentSpec Stage 2: Configure Perception Processing / Sensor Fusion Pipeline.
template <typename ModelType, typename SensorSuiteType>
class AgentSpecPerceptionStage {
public:
    AgentSpecPerceptionStage(ModelType model, SensorSuiteType sensorSuite)
        : _model(std::move(model)), _sensorSuite(std::move(sensorSuite)) {}

    /// @brief Configure user-defined perception post-processing or sensor fusion pipeline.
    template <std::size_t OutputSize, typename FusionFn>
    auto withPerceptionChain(FusionFn&& fusionFn)
    {
        auto pipeline = makePerceptionPipeline<OutputSize>(
            std::move(_sensorSuite), std::forward<FusionFn>(fusionFn)
        );
        return AgentSpecActuatorsStage<ModelType, decltype(pipeline)>(
            std::move(_model), std::move(pipeline)
        );
    }

    /// @brief Use raw sensor suite observations directly without sensor fusion.
    auto withoutPerceptionChain()
    {
        return AgentSpecActuatorsStage<ModelType, SensorSuiteType>(
            std::move(_model), std::move(_sensorSuite)
        );
    }

    /// @brief Attach physical motor drives / actuators suite directly (bypassing perception chain).
    template <concepts::Actuator... Actuators>
    auto withActuators(Actuators... actuators)
    {
        return withoutPerceptionChain().withActuators(std::move(actuators)...);
    }

private:
    ModelType _model;
    SensorSuiteType _sensorSuite;
};

/// @brief Fluent AgentSpec Stage 1: Attach Multi-Modal Sensor Suite.
template <typename ModelType>
class AgentSpecSensorsStage {
public:
    explicit AgentSpecSensorsStage(ModelType model) : _model(std::move(model)) {}

    /// @brief Attach multi-modal sensor suite (preset and custom sensors).
    template <concepts::Sensor... Sensors>
    auto withSensors(Sensors... sensors)
    {
        SensorSuite<Sensors...> sensorSuite(std::move(sensors)...);
        return AgentSpecPerceptionStage<ModelType, SensorSuite<Sensors...>>(
            std::move(_model), std::move(sensorSuite)
        );
    }

private:
    ModelType _model;
};

/// @brief Fluent AgentSpec Stage 0: Define Agent Model (CAD / Entity / Mass).
class AgentSpecModelStage {
public:
    AgentSpecModelStage() = default;

    /// @brief Define agent physical model entity (mesh, mass, initial properties).
    template <typename ModelType>
    auto withModel(ModelType model)
    {
        return AgentSpecSensorsStage<ModelType>(std::move(model));
    }
};

/// @brief Entry point for constructing reusable decoupled agent specifications.
[[nodiscard]] constexpr auto makeAgentSpec()
{
    return AgentSpecModelStage{};
}

} // namespace corium_sim::agent
