#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <tuple>
#include <utility>

#include <corium/IEventSink.hpp>
#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/scene/SimEntity.hpp"

namespace corium_sim::agent {

/// @brief Compile-time variadic container for actuator suites with zero dynamic memory allocation.
/// @tparam Actuators Actuator types satisfying concepts::Actuator.
template <concepts::Actuator... Actuators>
class ActuatorSuite {
public:
    /// @brief Total aggregated action dimension resolved at compile time.
    static constexpr std::size_t total_action_size = (Actuators::action_size + ... + 0);

    using ActionBuffer = std::array<float, total_action_size>;

    constexpr ActuatorSuite() = default;
    constexpr explicit ActuatorSuite(Actuators... actuators) : _actuators(std::move(actuators)...) {}

    /// @brief Apply aggregated action vector to target entity across all actuators.
    /// @param entity Target physical entity.
    /// @param action Aggregated action vector.
    /// @param eventSink Optional Corium event sink handle for event-driven telemetry and tracing.
    /// @param stepIndex Current simulation step index.
    inline void apply(
        scene::SimEntity& entity,
        std::span<const float> action,
        corium::IEventSinkT<DefaultSimEvents> eventSink = {},
        uint64_t stepIndex = 0
    ) noexcept
    {
        if constexpr (total_action_size == 0) {
            return;
        }

        std::size_t offset = 0;
        std::apply([&](auto&... actuator) {
            (([&]() {
                using ActuatorType = std::decay_t<decltype(actuator)>;
                constexpr std::size_t size = ActuatorType::action_size;
                if (offset + size <= action.size()) {
                    auto actSubspan = action.subspan(offset, size);
                    actuator.apply(entity, actSubspan);

                    if (eventSink) {
                        std::vector<float> actVec(actSubspan.begin(), actSubspan.end());
                        eventSink.post(ActuatorAppliedEvent{
                            .actuatorName = entity.name.empty() ? "actuator" : (entity.name + "_actuator"),
                            .actuatorType = typeid(ActuatorType).name(),
                            .stepIndex = stepIndex,
                            .actions = std::move(actVec)
                        });

                        if constexpr (size >= 2) {
                            eventSink.post(AgentActionCommand{
                                .agentId = entity.id,
                                .moveForward = actSubspan[0],
                                .turnYaw = actSubspan[1],
                                .moveUp = (size >= 3 ? actSubspan[2] : 0.0f),
                                .resetEpisode = false
                            });
                        }
                    }
                }
                offset += size;
            }()), ...);
        }, _actuators);
    }

    /// @brief Access a specific actuator by index at compile time.
    template <std::size_t Index>
    [[nodiscard]] constexpr decltype(auto) getActuator() noexcept
    {
        return std::get<Index>(_actuators);
    }

    /// @brief Access a specific actuator by index at compile time (const).
    template <std::size_t Index>
    [[nodiscard]] constexpr decltype(auto) getActuator() const noexcept
    {
        return std::get<Index>(_actuators);
    }

private:
    std::tuple<Actuators...> _actuators{};
};

} // namespace corium_sim::agent
