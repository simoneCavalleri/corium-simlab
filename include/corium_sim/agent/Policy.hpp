#pragma once

#include <concepts>
#include <random>
#include <span>
#include <type_traits>
#include "corium_sim/agent/Concepts.hpp"

namespace corium_sim::agent {

namespace concepts {

/// @brief C++20 Concept for User-Defined Agent Planning & Policy Logic.
/// A Policy accepts an ObservationBuffer from an Agent and returns an ActionBuffer.
template <typename T, typename Agent>
concept Policy = requires(T policy, const typename Agent::ObservationBuffer& obs) {
    { policy.plan(obs) } -> std::convertible_to<typename Agent::ActionBuffer>;
};

} // namespace concepts

/// @brief Generic Random Exploration Policy for continuous action spaces.
template <typename Agent>
class RandomPolicy {
public:
    using ActionBuffer = typename Agent::ActionBuffer;
    using ObservationBuffer = typename Agent::ObservationBuffer;

    explicit RandomPolicy(float minVal = -1.0f, float maxVal = 1.0f)
        : _dist(minVal, maxVal)
    {
        std::random_device rd;
        _rng = std::mt19937(rd());
    }

    [[nodiscard]] inline ActionBuffer plan([[maybe_unused]] const ObservationBuffer& obs) noexcept
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
template <typename Agent, typename PlanFunc>
class CustomPolicy {
public:
    using ActionBuffer = typename Agent::ActionBuffer;
    using ObservationBuffer = typename Agent::ObservationBuffer;

    explicit CustomPolicy(PlanFunc func) : _func(std::move(func)) {}

    [[nodiscard]] inline ActionBuffer plan(const ObservationBuffer& obs) noexcept
    {
        return _func(obs);
    }

private:
    PlanFunc _func;
};

} // namespace corium_sim::agent
