#pragma once

#include <cstddef>
#include <span>
#include <utility>

#include <corium/IEventSink.hpp>
#include "corium_sim/agent/ActuatorSuite.hpp"
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/agent/SensorSuite.hpp"
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent {

/// @brief Monomorphized Physical Agent combining Body, SensorSuite, and ActuatorSuite at compile time.
/// @tparam BodyType Physical body representation (default scene::SimEntity).
/// @tparam SensorSuiteType Compile-time sensor suite.
/// @tparam ActuatorSuiteType Compile-time actuator suite.
template <
    typename BodyType,
    typename SensorSuiteType,
    typename ActuatorSuiteType
>
class PhysicalAgent {
public:
    using ObservationBuffer = typename SensorSuiteType::ObservationBuffer;
    using ActionBuffer = typename ActuatorSuiteType::ActionBuffer;

    static constexpr std::size_t observation_size = SensorSuiteType::total_observation_size;
    static constexpr std::size_t action_size = ActuatorSuiteType::total_action_size;

    constexpr PhysicalAgent() = default;

    constexpr PhysicalAgent(BodyType body, SensorSuiteType sensors, ActuatorSuiteType actuators)
        : _body(std::move(body)), _sensors(std::move(sensors)), _actuators(std::move(actuators)) {}

    /// @brief Access underlying physical body entity (mutable).
    [[nodiscard]] inline BodyType& body() noexcept { return _body; }

    /// @brief Access underlying physical body entity (const).
    [[nodiscard]] inline const BodyType& body() const noexcept { return _body; }

    /// @brief Access sensor suite (mutable).
    [[nodiscard]] inline SensorSuiteType& sensors() noexcept { return _sensors; }

    /// @brief Access sensor suite (const).
    [[nodiscard]] inline const SensorSuiteType& sensors() const noexcept { return _sensors; }

    /// @brief Access actuator suite (mutable).
    [[nodiscard]] inline ActuatorSuiteType& actuators() noexcept { return _actuators; }

    /// @brief Access actuator suite (const).
    [[nodiscard]] inline const ActuatorSuiteType& actuators() const noexcept { return _actuators; }

    /// @brief Sample all sensors against surrounding 3D environment scene into zero-copy observation buffer.
    [[nodiscard]] inline ObservationBuffer observe(
        const scene::SimScene& scene,
        renderer::WebGpuBackend* gpuBackend = nullptr,
        corium::IEventSinkT<DefaultSimEvents> eventSink = {},
        uint64_t stepIndex = 0
    ) noexcept
    {
        return _sensors.observe(_body, scene, gpuBackend, eventSink, stepIndex);
    }

    /// @brief Apply aggregated action vector to agent actuators.
    inline void actuate(
        std::span<const float> action,
        corium::IEventSinkT<DefaultSimEvents> eventSink = {},
        uint64_t stepIndex = 0
    ) noexcept
    {
        _actuators.apply(_body, action, eventSink, stepIndex);
    }

private:
    BodyType _body{};
    SensorSuiteType _sensors{};
    ActuatorSuiteType _actuators{};
};

/// @brief Fluent Builder for assembling compile-time Physical Agents.
template <typename BodyType = scene::SimEntity>
struct AgentBuilder {
    template <typename... SensorTypes>
    struct WithSensors {
        template <typename... ActuatorTypes>
        struct WithActuators {
            using SensorSuiteType = SensorSuite<SensorTypes...>;
            using ActuatorSuiteType = ActuatorSuite<ActuatorTypes...>;
            using Type = PhysicalAgent<BodyType, SensorSuiteType, ActuatorSuiteType>;
        };
    };
};

} // namespace corium_sim::agent
