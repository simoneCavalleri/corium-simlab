#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <corium/Service.hpp>
#include <corium/ServiceRegistry.hpp>
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::services {

// Forward declarations
class PlannerService;
class ActuatorService;

/// @brief Synchronous Service managing Agent Onboard Sensors.
/// Listens to SenseTriggerEvent, samples environment observations, and sends AgentPerceptionEvent
/// directly to PlannerService via sendToService<PlannerService>(...).
class SensorService : public corium::Service<DefaultSimEvents> {
public:
    using SampleFunc = std::function<std::vector<float>(uint64_t stepIndex)>;

    SensorService()
    {
        on([this](const SenseTriggerEvent& evt) {
            std::vector<float> obs{};
            if (_sampleFunc) {
                obs = _sampleFunc(evt.stepIndex);
            }

            // Post telemetry trace event to main event bus
            postEvent(SensorSampleEvent{
                .sensorName = "sensor_service",
                .sensorType = "SensorService",
                .stepIndex = evt.stepIndex,
                .observations = obs
            });

            // Direct Service-to-Service Communication: Send perception to PlannerService!
            sendToService<PlannerService>(AgentPerceptionEvent{
                .agentId = evt.agentId,
                .stepIndex = evt.stepIndex,
                .observations = std::move(obs)
            });
        });
    }

    void setSampleFunc(SampleFunc sampleFunc) noexcept
    {
        _sampleFunc = std::move(sampleFunc);
    }

private:
    SampleFunc _sampleFunc{};
};

/// @brief Synchronous Service managing Agent AI / Policy Planning.
/// Listens to AgentPerceptionEvent from SensorService, evaluates planning policy,
/// and sends AgentActionCommand directly to ActuatorService via sendToService<ActuatorService>(...).
class PlannerService : public corium::Service<DefaultSimEvents> {
public:
    using PlanFunc = std::function<std::vector<float>(const std::vector<float>& obs)>;

    PlannerService()
    {
        on([this](const AgentPerceptionEvent& evt) {
            std::vector<float> action{};
            if (_planFunc) {
                action = _planFunc(evt.observations);
            }

            float fwd = action.size() > 0 ? action[0] : 0.0f;
            float yaw = action.size() > 1 ? action[1] : 0.0f;
            float up  = action.size() > 2 ? action[2] : 0.0f;

            // Direct Service-to-Service Communication: Send action command to ActuatorService!
            sendToService<ActuatorService>(AgentActionCommand{
                .agentId = evt.agentId,
                .moveForward = fwd,
                .turnYaw = yaw,
                .moveUp = up,
                .resetEpisode = false,
                .stepIndex = evt.stepIndex,
                .actions = std::move(action)
            });
        });
    }

    void setPlanFunc(PlanFunc planFunc) noexcept
    {
        _planFunc = std::move(planFunc);
    }

private:
    PlanFunc _planFunc{};
};

/// @brief Synchronous Service managing Agent Actuators and Motion Controllers.
/// Listens to AgentActionCommand from PlannerService, applies actions to physical entity,
/// and posts ActuatorAppliedEvent back to the main EventBus.
class ActuatorService : public corium::Service<DefaultSimEvents> {
public:
    using ApplyFunc = std::function<void(std::span<const float> actions, uint64_t stepIndex)>;

    ActuatorService()
    {
        on([this](const AgentActionCommand& cmd) {
            if (_applyFunc) {
                std::span<const float> actSpan(cmd.actions.data(), cmd.actions.size());
                _applyFunc(actSpan, cmd.stepIndex);
            }

            // Post telemetry trace event back to main event bus
            postEvent(ActuatorAppliedEvent{
                .actuatorName = "actuator_service",
                .actuatorType = "ActuatorService",
                .stepIndex = cmd.stepIndex,
                .actions = cmd.actions
            });
        });
    }

    void setApplyFunc(ApplyFunc applyFunc) noexcept
    {
        _applyFunc = std::move(applyFunc);
    }

private:
    ApplyFunc _applyFunc{};
};

} // namespace corium_sim::services
