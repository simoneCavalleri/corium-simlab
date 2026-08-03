#pragma once

#include <cstddef>
#include <future>
#include <memory>
#include <thread>
#include <vector>

namespace corium_sim::environment {

/// @brief Multithreaded Environment Pool for parallel vector environment stepping.
/// Executes $N$ simulation environment instances concurrently across CPU worker threads
/// for high-throughput Reinforcement Learning (RL) training (e.g. PPO / SAC batch collection).
/// @tparam EnvType Simulation environment type satisfying environment requirements.
template <typename EnvType>
class ParallelEnvironmentPool {
public:
    explicit ParallelEnvironmentPool(std::size_t numEnvironments = 4)
        : _environments(numEnvironments) {}

    /// @brief Reset all parallel simulation environments concurrently.
    void resetAll()
    {
        std::vector<std::future<void>> futures;
        futures.reserve(_environments.size());

        for (auto& env : _environments) {
            futures.push_back(std::async(std::launch::async, [&env]() {
                env.reset();
            }));
        }

        for (auto& f : futures) {
            f.get();
        }
    }

    /// @brief Step all parallel simulation environments concurrently by dt seconds.
    void stepAll(float dt = 0.01667f)
    {
        std::vector<std::future<void>> futures;
        futures.reserve(_environments.size());

        for (auto& env : _environments) {
            futures.push_back(std::async(std::launch::async, [&env, dt]() {
                env.step(dt);
            }));
        }

        for (auto& f : futures) {
            f.get();
        }
    }

    /// @brief Access a specific environment instance by index (mutable).
    [[nodiscard]] EnvType& getEnv(std::size_t index) noexcept
    {
        return _environments[index];
    }

    /// @brief Access a specific environment instance by index (const).
    [[nodiscard]] const EnvType& getEnv(std::size_t index) const noexcept
    {
        return _environments[index];
    }

    [[nodiscard]] std::size_t size() const noexcept { return _environments.size(); }

private:
    std::vector<EnvType> _environments{};
};

} // namespace corium_sim::environment
