#pragma once

// =============================================================================
// Corium SimLab — Simulation Environment Configuration
// =============================================================================

#include <cstdint>
#include "corium_sim/math/Math.hpp"

namespace corium_sim {

/// @brief Configuration parameters for the Corium SimLab simulation environment.
/// Controls episode length, reward shaping, sensor resolution, and physics settings.
struct SimConfig {
    /// Maximum number of simulation steps per training episode before truncation.
    uint32_t maxEpisodeSteps = 500;

    /// Resolution of the onboard agent visual sensor camera (width x height pixels).
    uint32_t sensorWidth = 128;
    uint32_t sensorHeight = 128;

    /// Distance threshold (meters) below which the agent is considered to have reached the target.
    float reachThreshold = 1.5f;

    /// Bonus reward added when the agent reaches the target (distance < reachThreshold).
    float reachBonus = 100.0f;

    /// If true, apply gravity to non-static entities.
    bool enableGravity = false;

    /// Gravity acceleration vector (used only when enableGravity is true).
    math::Vec3 gravity{0.0f, -9.81f, 0.0f};

    /// Fixed timestep for physics integration in headless/Python mode (seconds).
    float fixedTimestep = 0.016667f;
};

} // namespace corium_sim
