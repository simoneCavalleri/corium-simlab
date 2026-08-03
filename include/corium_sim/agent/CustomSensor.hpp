#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <utility>

#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent::sensors {

/// @brief Helper template for defining inline environment-aware custom sensors via lambda or callback function.
/// @tparam Size Compile-time observation dimension of the custom sensor.
/// @tparam SampleFunc Function or Lambda type with signature `std::array<float, Size>(const scene::SimEntity&, const scene::SimScene&)`.
template <std::size_t Size, typename SampleFunc>
class CustomSensor {
public:
    static constexpr std::size_t observation_size = Size;

    explicit CustomSensor(SampleFunc func) : _func(std::move(func)) {}

    [[nodiscard]] inline std::span<const float> sample(const scene::SimEntity& entity, const scene::SimScene& scene) noexcept
    {
        _buffer = _func(entity, scene);
        return std::span<const float>(_buffer.data(), Size);
    }

private:
    SampleFunc _func;
    std::array<float, Size> _buffer{};
};

/// @brief Helper function for creating an environment-aware CustomSensor with deduction of lambda type.
template <std::size_t Size, typename SampleFunc>
[[nodiscard]] constexpr auto makeCustomSensor(SampleFunc&& func)
{
    return CustomSensor<Size, std::decay_t<SampleFunc>>(std::forward<SampleFunc>(func));
}

} // namespace corium_sim::agent::sensors
