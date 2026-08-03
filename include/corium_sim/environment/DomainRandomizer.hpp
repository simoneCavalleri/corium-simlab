#pragma once

#include <cmath>
#include <random>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::environment {

/// @brief Configuration settings for Sim-to-Real Domain Randomization.
struct DomainRandomizationConfig {
    bool enablePoseRandomization = true;
    float positionStdDev = 0.1f;        // Position perturbation std dev (meters)
    float rotationStdDev = 5.0f;        // Rotation perturbation std dev (degrees)

    bool enablePhysicsRandomization = true;
    float massScaleRange = 0.2f;        // Mass scaling +/- 20%
    float frictionScaleRange = 0.15f;   // Friction scaling +/- 15%

    bool enableSensorNoise = true;
    float sensorNoiseStdDev = 0.02f;    // Gaussian sensor noise std dev
};

/// @brief Sim-to-Real Domain Randomization Engine.
/// Applies stochastic perturbations to initial entity poses, physical mass/friction properties,
/// and sensor observations upon environment reset.
class DomainRandomizer {
public:
    DomainRandomizer() : _gen(_rd()) {}

    explicit DomainRandomizer(DomainRandomizationConfig config)
        : _config(config), _gen(_rd()) {}

    /// @brief Randomize scene initial poses and physical properties.
    void randomizeScene(scene::SimScene& scene) noexcept
    {
        if (!_config.enablePoseRandomization && !_config.enablePhysicsRandomization) {
            return;
        }

        std::normal_distribution<float> posDist(0.0f, _config.positionStdDev);
        std::normal_distribution<float> rotDist(0.0f, _config.rotationStdDev);
        std::uniform_real_distribution<float> massDist(1.0f - _config.massScaleRange, 1.0f + _config.massScaleRange);

        for (auto& entity : scene.entities()) {
            if (!entity.isStatic) {
                if (_config.enablePoseRandomization) {
                    entity.position.x += posDist(_gen);
                    entity.position.z += posDist(_gen);
                    entity.rotation.y += rotDist(_gen);
                }
                if (_config.enablePhysicsRandomization) {
                    entity.mass *= massDist(_gen);
                }
            }
        }
    }

    /// @brief Add Gaussian noise to a raw float sensor reading vector.
    template <std::size_t Size>
    [[nodiscard]] std::array<float, Size> applySensorNoise(const std::array<float, Size>& rawObs) noexcept
    {
        if (!_config.enableSensorNoise || _config.sensorNoiseStdDev <= 0.0f) {
            return rawObs;
        }

        std::array<float, Size> noisyObs = rawObs;
        std::normal_distribution<float> noiseDist(0.0f, _config.sensorNoiseStdDev);

        for (std::size_t i = 0; i < Size; ++i) {
            noisyObs[i] += noiseDist(_gen);
        }
        return noisyObs;
    }

    [[nodiscard]] DomainRandomizationConfig& config() noexcept { return _config; }
    [[nodiscard]] const DomainRandomizationConfig& config() const noexcept { return _config; }

private:
    DomainRandomizationConfig _config{};
    std::random_device _rd{};
    std::mt19937 _gen;
};

} // namespace corium_sim::environment
