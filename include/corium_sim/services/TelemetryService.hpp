#pragma once

#include <chrono>
#include <stop_token>
#include <thread>

#include "corium_sim/events/SimEvents.hpp"
#include <corium/BackgroundService.hpp>

namespace corium_sim::services {

/// @brief High-frequency telemetry background service streaming sensor data & agent poses.
/// Runs on a dedicated std::jthread and posts events directly to Corium's Lock-Free MPSC queue.
/// Zero dynamic heap allocations, zero locks on the main thread.
template <typename EventVariant = DefaultSimEvents>
class TelemetryService : public corium::BackgroundService<EventVariant> {
public:
    TelemetryService() = default;

    void setFrequency(std::chrono::milliseconds interval) noexcept
    {
        _interval = interval;
    }

    void run(std::stop_token stopToken)
    {
        uint32_t stepCounter = 0;
        double timestamp = 0.0;

        while (!stopToken.stop_requested()) {
            // Post Telemetry Sensor Event
            this->postEvent(TelemetryDataEvent{
                .sensorId = 1,
                .timestamp = timestamp,
                .value = static_cast<float>(stepCounter % 100) * 0.1f,
                .sampleCount = stepCounter
            });

            // Post Agent Pose Update Event
            this->postEvent(AgentPoseEvent{
                .agentId = 101,
                .posX = static_cast<float>(stepCounter) * 0.05f,
                .posY = 0.0f,
                .posZ = static_cast<float>(stepCounter % 50) * 0.1f,
                .rotX = 0.0f,
                .rotY = 0.0f,
                .rotZ = 0.0f,
                .rotW = 1.0f
            });

            stepCounter++;
            timestamp += std::chrono::duration<double>(_interval).count();
            std::this_thread::sleep_for(_interval);
        }
    }

private:
    std::chrono::milliseconds _interval{50}; // Default 20 Hz
};

} // namespace corium_sim::services
