#pragma once

#include <concepts>
#include <cstddef>
#include <span>
#include <type_traits>
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::renderer { class WebGpuBackend; }

namespace corium_sim::agent::concepts {

/// @brief C++20 Concept for Environment-Aware Physical Sensors.
/// Sensors receive both the agent's physical body entity, the surrounding 3D scene environment,
/// and an optional WebGPU render backend pointer for GPU-accelerated sensing (cameras, GPU raycasting).
template <typename T>
concept Sensor = requires(T sensor, const scene::SimEntity& entity, const scene::SimScene& scene, renderer::WebGpuBackend* gpuBackend) {
    { T::observation_size } -> std::convertible_to<std::size_t>;
    requires requires {
        { sensor.sample(entity, scene) } -> std::convertible_to<std::span<const float>>;
    } || requires {
        { sensor.sample(entity, scene, gpuBackend) } -> std::convertible_to<std::span<const float>>;
    };
};

/// @brief C++20 Concept for compile-time Physical Actuators.
template <typename T>
concept Actuator = requires(T actuator, scene::SimEntity& entity, std::span<const float> action) {
    { T::action_size } -> std::convertible_to<std::size_t>;
    { actuator.apply(entity, action) };
};

/// @brief C++20 Concept for Simulation Environments.
template <typename T>
concept Environment = requires(T env) {
    { env.step() };
    { env.reset() };
};

} // namespace corium_sim::agent::concepts
