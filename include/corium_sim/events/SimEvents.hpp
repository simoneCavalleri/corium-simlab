#pragma once

#include <cstdint>
#include <variant>
#include <corium/Events.hpp>
#include <corium/ui/WindowEvents.hpp>

namespace corium_sim {

/// @brief Event emitted when high-frequency telemetry/sensor data is received from a simulation background thread.
struct TelemetryDataEvent {
    uint32_t sensorId = 0;
    double timestamp = 0.0;
    float value = 0.0f;
    uint32_t sampleCount = 0;
};

/// @brief Event emitted when an autonomous agent/robot pose is updated in the simulation world.
struct AgentPoseEvent {
    uint32_t agentId = 0;
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
    float rotX = 0.0f, rotY = 0.0f, rotZ = 0.0f, rotW = 1.0f;
};

/// @brief Event emitted when the 3D viewport camera position or orientation is modified.
struct CameraUpdateEvent {
    float eyeX = 0.0f, eyeY = 5.0f, eyeZ = 10.0f;
    float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f;
    float fovDegrees = 60.0f;
};

/// @brief Event emitted on each physics / simulation logic step.
struct SimStepEvent {
    uint64_t stepIndex = 0;
    double deltaTime = 0.0;
};

/// @brief Combined default variant list of core Corium events, UI events, and Simulation events.
using DefaultSimEvents = std::variant<
    corium::QuitEvent,
    corium::TickEvent,
    corium::UpdateEvent,
    corium::ErrorEvent,
    corium::SignalEvent,
    corium::ui::WindowResizeEvent,
    corium::ui::MouseMoveEvent,
    corium::ui::MouseButtonEvent,
    corium::ui::KeyEvent,
    corium::ui::WindowFocusEvent,
    corium::ui::WindowCloseEvent,
    TelemetryDataEvent,
    AgentPoseEvent,
    CameraUpdateEvent,
    SimStepEvent
>;

} // namespace corium_sim
