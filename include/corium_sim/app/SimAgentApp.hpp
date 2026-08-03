#pragma once

#include <memory>
#include <stop_token>
#include <utility>

#include <corium/AppCore.hpp>
#include <corium/ServiceRegistry.hpp>
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/events/SimEventTracer.hpp"
#include "corium_sim/services/SimServices.hpp"

namespace corium_sim::app {

/// @brief Configurable Corium Agent Application encapsulation.
/// Manages SensorService, PlannerService, ActuatorService, and SimEventTracer
/// as a first-class Corium AppCore.
class ConfigurableSimAgentApp : public corium::AppCoreT<ConfigurableSimAgentApp, SimRuntime::EventBusType> {
public:
    using BaseApp = corium::AppCoreT<ConfigurableSimAgentApp, SimRuntime::EventBusType>;

    ConfigurableSimAgentApp() = default;

    [[nodiscard]] services::SensorService& sensorService() noexcept { return _sensorService; }
    [[nodiscard]] const services::SensorService& sensorService() const noexcept { return _sensorService; }

    [[nodiscard]] services::PlannerService& plannerService() noexcept { return _plannerService; }
    [[nodiscard]] const services::PlannerService& plannerService() const noexcept { return _plannerService; }

    [[nodiscard]] services::ActuatorService& actuatorService() noexcept { return _actuatorService; }
    [[nodiscard]] const services::ActuatorService& actuatorService() const noexcept { return _actuatorService; }

    [[nodiscard]] events::SimEventTracer& tracer() noexcept { return _tracer; }
    [[nodiscard]] const events::SimEventTracer& tracer() const noexcept { return _tracer; }

    /// @brief Lifecycle hook called by Corium Runtime: Registers SensorService, PlannerService, and ActuatorService.
    void onConfigureServices(typename BaseApp::ServiceRegistryType& registry)
    {
        registry.registerService(_sensorService);
        registry.registerService(_plannerService);
        registry.registerService(_actuatorService);
    }

    /// @brief Lifecycle hook called by Corium Runtime: Registers telemetry event tracer on the agent's event bus.
    void onRegisterHandlers()
    {
        _tracer.registerWith(this->events());
    }



    /// @brief Execute a 100% deterministic lock-step inter-service step cycle:
    ///        SenseTriggerEvent -> SensorService -> AgentPerceptionEvent -> PlannerService -> AgentActionCommand -> ActuatorService -> ActuatorAppliedEvent.
    void step(uint64_t stepIndex, float dt = 0.01667f)
    {
        // 1. Post SimStepEvent & SenseTriggerEvent to main event bus & SensorService
        this->events().post(SimStepEvent{.stepIndex = stepIndex, .deltaTime = static_cast<double>(dt)});
        _sensorService.serviceSink().post(SenseTriggerEvent{.agentId = 1, .stepIndex = stepIndex});

        // 2. Lock-Step Inter-Service Execution Sequence
        _sensorService.pump();   // SensorService -> sendToService<PlannerService>(AgentPerceptionEvent)
        _plannerService.pump();  // PlannerService -> sendToService<ActuatorService>(AgentActionCommand)
        _actuatorService.pump(); // ActuatorService -> postEvent(ActuatorAppliedEvent) & apply actuation

        // 3. Process remaining events on main EventBus
        while (this->events().processOne()) {}
    }

private:
    services::SensorService _sensorService;
    services::PlannerService _plannerService;
    services::ActuatorService _actuatorService;
    events::SimEventTracer _tracer;
};

/// @brief Templated static CRTP base class for custom Agent Applications deriving from Corium AppCore.
template <typename Derived, typename AgentSpecType = void>
class SimAgentApp : public corium::AppCoreT<Derived, SimRuntime::EventBusType> {
public:
    using BaseApp = corium::AppCoreT<Derived, SimRuntime::EventBusType>;

    SimAgentApp() = default;

    [[nodiscard]] services::SensorService& sensorService() noexcept { return _sensorService; }
    [[nodiscard]] const services::SensorService& sensorService() const noexcept { return _sensorService; }

    [[nodiscard]] services::PlannerService& plannerService() noexcept { return _plannerService; }
    [[nodiscard]] const services::PlannerService& plannerService() const noexcept { return _plannerService; }

    [[nodiscard]] services::ActuatorService& actuatorService() noexcept { return _actuatorService; }
    [[nodiscard]] const services::ActuatorService& actuatorService() const noexcept { return _actuatorService; }

    [[nodiscard]] events::SimEventTracer& tracer() noexcept { return _tracer; }
    [[nodiscard]] const events::SimEventTracer& tracer() const noexcept { return _tracer; }

    void onConfigureServices(typename BaseApp::ServiceRegistryType& registry)
    {
        registry.registerService(_sensorService);
        registry.registerService(_plannerService);
        registry.registerService(_actuatorService);
    }

    void onRegisterHandlers()
    {
        _tracer.registerWith(this->events());
    }

    void step(uint64_t stepIndex, float dt = 0.01667f)
    {
        this->events().post(SimStepEvent{.stepIndex = stepIndex, .deltaTime = static_cast<double>(dt)});
        _sensorService.serviceSink().post(SenseTriggerEvent{.agentId = 1, .stepIndex = stepIndex});

        _sensorService.pump();
        _plannerService.pump();
        _actuatorService.pump();

        while (this->events().processOne()) {}
    }

private:
    services::SensorService _sensorService;
    services::PlannerService _plannerService;
    services::ActuatorService _actuatorService;
    events::SimEventTracer _tracer;
};

} // namespace corium_sim::app
