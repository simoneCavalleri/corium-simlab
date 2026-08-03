#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <span>
#include <vector>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/environment/Task.hpp"

namespace corium_sim::environment {

/// @brief Abstract base class for single reward terms in RL reward engineering.
class IRewardTerm {
public:
    virtual ~IRewardTerm() = default;

    /// @brief Calculate reward scalar component.
    /// @param agentPos Current 3D position of agent.
    /// @param targetPos Target 3D goal position.
    /// @param action Current action vector applied.
    /// @param isCollided True if agent collided with obstacles.
    [[nodiscard]] virtual float compute(
        const math::Vec3& agentPos,
        const math::Vec3& targetPos,
        std::span<const float> action,
        bool isCollided
    ) noexcept = 0;
};

/// @brief Reward term penalizing distance to 3D target goal.
class DistanceToGoalPenalty : public IRewardTerm {
public:
    explicit DistanceToGoalPenalty(float weight = 1.0f) : _weight(weight) {}

    [[nodiscard]] float compute(
        const math::Vec3& agentPos,
        const math::Vec3& targetPos,
        [[maybe_unused]] std::span<const float> action,
        [[maybe_unused]] bool isCollided
    ) noexcept override
    {
        float dist = (agentPos - targetPos).length();
        return -_weight * dist;
    }

private:
    float _weight = 1.0f;
};

/// @brief Reward term granting large positive bonus when goal is reached.
class GoalReachedBonus : public IRewardTerm {
public:
    explicit GoalReachedBonus(float bonus = 100.0f, float threshold = 0.5f)
        : _bonus(bonus), _threshold(threshold) {}

    [[nodiscard]] float compute(
        const math::Vec3& agentPos,
        const math::Vec3& targetPos,
        [[maybe_unused]] std::span<const float> action,
        [[maybe_unused]] bool isCollided
    ) noexcept override
    {
        float dist = (agentPos - targetPos).length();
        return (dist < _threshold) ? _bonus : 0.0f;
    }

private:
    float _bonus = 100.0f;
    float _threshold = 0.5f;
};

/// @brief Reward term penalizing high control action magnitude for smooth motion.
class ActionSmoothingPenalty : public IRewardTerm {
public:
    explicit ActionSmoothingPenalty(float weight = 0.01f) : _weight(weight) {}

    [[nodiscard]] float compute(
        [[maybe_unused]] const math::Vec3& agentPos,
        [[maybe_unused]] const math::Vec3& targetPos,
        std::span<const float> action,
        [[maybe_unused]] bool isCollided
    ) noexcept override
    {
        float normSq = 0.0f;
        for (float a : action) {
            normSq += a * a;
        }
        return -_weight * normSq;
    }

private:
    float _weight = 0.01f;
};

/// @brief Reward term penalizing unwanted collisions.
class CollisionPenalty : public IRewardTerm {
public:
    explicit CollisionPenalty(float penalty = 50.0f) : _penalty(penalty) {}

    [[nodiscard]] float compute(
        [[maybe_unused]] const math::Vec3& agentPos,
        [[maybe_unused]] const math::Vec3& targetPos,
        [[maybe_unused]] std::span<const float> action,
        bool isCollided
    ) noexcept override
    {
        return isCollided ? -_penalty : 0.0f;
    }

private:
    float _penalty = 50.0f;
};

/// @brief Fluent Builder for composing multi-objective RL reward functions.
/// Also implements ITask so it can be used as the canonical reward + termination system.
class RewardBuilder : public ITask {
public:
    RewardBuilder() = default;

    // -------------------------------------------------------------------------
    // Configuration (fluent)
    // -------------------------------------------------------------------------

    /// @brief Set the 3D goal position used for distance-based reward and done check.
    RewardBuilder& withTarget(math::Vec3 target) noexcept
    {
        _target = target;
        return *this;
    }

    /// @brief Set the distance threshold below which the episode is considered done.
    RewardBuilder& withGoalThreshold(float threshold) noexcept
    {
        _goalThreshold = threshold;
        return *this;
    }

    /// @brief Add a reward term instance to the composition pipeline.
    RewardBuilder& addTerm(std::shared_ptr<IRewardTerm> term)
    {
        if (term) {
            _terms.push_back(std::move(term));
        }
        return *this;
    }

    /// @brief Convenience template to instantiate and add a reward term type.
    template <typename TermType, typename... Args>
    RewardBuilder& addTerm(Args&&... args)
    {
        _terms.push_back(std::make_shared<TermType>(std::forward<Args>(args)...));
        return *this;
    }

    // -------------------------------------------------------------------------
    // Context update (called by Incubator before computeResult)
    // -------------------------------------------------------------------------

    /// @brief Push current agent context so ITask::computeResult() has the data it needs.
    void updateContext(const math::Vec3& agentPos,
                       std::span<const float> action,
                       bool isCollided = false) noexcept
    {
        _agentPos   = agentPos;
        _lastAction = std::vector<float>(action.begin(), action.end());
        _isCollided = isCollided;
    }

    // -------------------------------------------------------------------------
    // Direct reward computation (legacy / Python path)
    // -------------------------------------------------------------------------

    /// @brief Evaluate aggregated total scalar reward across all composed terms.
    [[nodiscard]] float computeTotalReward(
        const math::Vec3& agentPos,
        const math::Vec3& targetPos,
        std::span<const float> action,
        bool isCollided = false
    ) noexcept
    {
        float total = 0.0f;
        for (const auto& term : _terms) {
            total += term->compute(agentPos, targetPos, action, isCollided);
        }
        return total;
    }

    // -------------------------------------------------------------------------
    // ITask implementation
    // -------------------------------------------------------------------------

    /// @brief Compute step reward, done flag, and truncated flag from stored context.
    [[nodiscard]] TaskStepResult computeResult() noexcept override
    {
        std::span<const float> actionSpan(_lastAction.data(), _lastAction.size());
        float reward = computeTotalReward(_agentPos, _target, actionSpan, _isCollided);

        float dist = (_agentPos - _target).length();
        bool done   = (dist < _goalThreshold);

        return TaskStepResult{
            .reward    = reward,
            .done      = done,
            .truncated = false,
            .info      = done ? "goal_reached" : ""
        };
    }

    /// @brief Reset transient episode state.
    void reset() noexcept override
    {
        _agentPos   = {};
        _lastAction.clear();
        _isCollided = false;
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    [[nodiscard]] const math::Vec3& target() const noexcept { return _target; }
    [[nodiscard]] float goalThreshold() const noexcept { return _goalThreshold; }

private:
    std::vector<std::shared_ptr<IRewardTerm>> _terms{};

    // Target configuration
    math::Vec3 _target{0.0f, 0.0f, 0.0f};
    float      _goalThreshold = 0.5f;

    // Transient context (set by updateContext each step)
    math::Vec3           _agentPos{};
    std::vector<float>   _lastAction{};
    bool                 _isCollided = false;
};

} // namespace corium_sim::environment
