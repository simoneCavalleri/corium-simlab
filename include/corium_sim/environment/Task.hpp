#pragma once

#include <cstdint>
#include <string>

namespace corium_sim::environment {

/// @brief Struct representing the step execution result of a simulation task.
struct TaskStepResult {
    float reward = 0.0f;
    bool done = false;
    bool truncated = false;
    std::string info{};
};

/// @brief Interface / Contract for defining custom training tasks, rewards, and reset criteria.
class ITask {
public:
    virtual ~ITask() = default;

    /// @brief Compute step reward, completion status (done), and termination flags.
    [[nodiscard]] virtual TaskStepResult computeResult() noexcept = 0;

    /// @brief Reset task state and internal target positions/goals.
    virtual void reset() noexcept = 0;
};

} // namespace corium_sim::environment
