#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <random>
#include <type_traits>
#include "corium_sim/agent/Concepts.hpp"

namespace corium_sim::agent {

namespace concepts {

/// @brief C++20 Concept for User-Defined Agent Planning & Policy Logic.
/// A Policy satisfies this concept if it has a plan() method that accepts any
/// observation buffer and returns a compatible action buffer.
template <typename T>
concept Policy = requires(T policy) {
    // plan() must be callable with at least some span/array-like type
    { policy.plan(std::declval<std::array<float, 1>>()) };
};

} // namespace concepts

/// @brief Generic Random Exploration Policy for fixed-size continuous action spaces.
/// @tparam ActionSize Number of action dimensions.
template <std::size_t ActionSize>
class RandomPolicy {
public:
    using ActionBuffer = std::array<float, ActionSize>;

    explicit RandomPolicy(float minVal = -1.0f, float maxVal = 1.0f)
        : _dist(minVal, maxVal)
    {
        std::random_device rd;
        _rng = std::mt19937(rd());
    }

    template <typename ObsBuffer>
    [[nodiscard]] inline ActionBuffer plan([[maybe_unused]] const ObsBuffer& obs) noexcept
    {
        ActionBuffer action{};
        for (auto& val : action) {
            val = _dist(_rng);
        }
        return action;
    }

private:
    std::mt19937 _rng;
    std::uniform_real_distribution<float> _dist;
};

/// @brief Custom Lambda / Function Wrapper Policy.
/// @tparam PlanFunc Callable with signature: ActionBuffer(const ObsBuffer&).
template <typename PlanFunc>
class CustomPolicy {
public:
    explicit CustomPolicy(PlanFunc func) : _func(std::move(func)) {}

    template <typename ObsBuffer>
    [[nodiscard]] inline auto plan(const ObsBuffer& obs) noexcept
    {
        return _func(obs);
    }

private:
    PlanFunc _func;
};

/// @brief Deduction guide for CustomPolicy via lambda.
template <typename PlanFunc>
CustomPolicy(PlanFunc) -> CustomPolicy<PlanFunc>;

} // namespace corium_sim::agent
