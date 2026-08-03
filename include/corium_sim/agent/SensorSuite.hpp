#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <span>
#include <tuple>
#include <utility>

#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent {

/// @brief Compile-time variadic container for sensor suites with zero dynamic memory allocation.
/// @tparam Sensors Sensor types satisfying concepts::Sensor.
template <concepts::Sensor... Sensors>
class SensorSuite {
public:
    /// @brief Total aggregated observation dimension resolved at compile time.
    static constexpr std::size_t total_observation_size = (Sensors::observation_size + ... + 0);

    using ObservationBuffer = std::array<float, total_observation_size>;

    constexpr SensorSuite() = default;
    constexpr explicit SensorSuite(Sensors... sensors) : _sensors(std::move(sensors)...) {}

    /// @brief Sample all sensors in sequence into a contiguous zero-copy float array.
    /// @param entity Target physical entity.
    /// @param scene Target 3D environment scene.
    /// @return Fixed-size array containing aggregated sensor observations.
    [[nodiscard]] inline ObservationBuffer observe(const scene::SimEntity& entity, const scene::SimScene& scene) noexcept
    {
        ObservationBuffer buffer{};
        if constexpr (total_observation_size == 0) {
            return buffer;
        }

        std::size_t offset = 0;
        std::apply([&](auto&... sensor) {
            (([&]() {
                auto obsSpan = sensor.sample(entity, scene);
                std::copy(obsSpan.begin(), obsSpan.end(), buffer.begin() + offset);
                offset += std::decay_t<decltype(sensor)>::observation_size;
            }()), ...);
        }, _sensors);

        return buffer;
    }

    /// @brief Access a specific sensor by index at compile time.
    template <std::size_t Index>
    [[nodiscard]] constexpr decltype(auto) getSensor() noexcept
    {
        return std::get<Index>(_sensors);
    }

    /// @brief Access a specific sensor by index at compile time (const).
    template <std::size_t Index>
    [[nodiscard]] constexpr decltype(auto) getSensor() const noexcept
    {
        return std::get<Index>(_sensors);
    }

private:
    std::tuple<Sensors...> _sensors{};
};

} // namespace corium_sim::agent
