#pragma once

#include <concepts>
#include <type_traits>
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::physics {

/// @brief C++20 Concept for Pluggable Physics Engines.
/// Any physics engine (Rigid Body, Soft Body, Kinematic, Bullet, PhysX, or Custom)
/// satisfying this concept can be plugged into SimEnvironment.
template <typename T>
concept PhysicsBackend = requires(T engine, scene::SimScene& scene, float dt) {
    { engine.step(scene, dt) };
    { engine.reset() };
};

} // namespace corium_sim::physics
