#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <corium/Events.hpp>
#include <corium/ui/WindowEvents.hpp>

#include <vector>
#include <corium/logging/logging.hpp>

namespace corium_sim {

/// @brief Event emitted to request loading a 3D asset into the simulation environment.
struct MeshLoadEvent {
    std::string assetPath{};
    uint32_t assetId = 0;
    float scale = 1.0f;
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
};

/// @brief Event emitted to command or position the 3D simulation camera.
struct CameraCommandEvent {
    float eyeX = 0.0f, eyeY = 5.0f, eyeZ = 10.0f;
    float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f;
    float fovDegrees = 60.0f;
};

/// @brief Event emitted when the 3D viewport camera state updates.
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

/// @brief Event emitted to reset the agent training environment state.
struct SimResetEvent {
    uint64_t episodeIndex = 0;
    uint64_t seed = 0;
};

/// @brief Event posted to trigger sensor sampling on agent components.
struct SenseTriggerEvent {
    uint32_t agentId = 0;
    uint64_t stepIndex = 0;
};

/// @brief Event posted when sensors finish sampling and publish perception observations.
struct AgentPerceptionEvent {
    uint32_t agentId = 0;
    uint64_t stepIndex = 0;
    std::vector<float> observations{};
};

/// @brief Event emitted to command an agent with linear and angular movement actions or raw action vector.
struct AgentActionCommand {
    uint32_t agentId = 0;
    float moveForward = 0.0f; // Linear velocity forward/backward
    float turnYaw = 0.0f;     // Angular velocity yaw
    float moveUp = 0.0f;      // Vertical velocity
    bool resetEpisode = false;
    uint64_t stepIndex = 0;
    std::vector<float> actions{};
};

/// @brief Zero-heap event emitted on each simulation step containing agent state, observations, and rewards.
struct AgentObservationEvent {
    uint32_t agentId = 0;
    float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
    float velX = 0.0f, velY = 0.0f, velZ = 0.0f;
    float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f;
    float distanceToTarget = 0.0f;
    float reward = 0.0f;
    bool isTerminated = false;
    bool isTruncated = false;
    uint64_t stepIndex = 0;
};

/// @brief Event emitted when an onboard visual sensor camera captures an offscreen frame payload.
struct SensorFrameEvent {
    uint32_t sensorId = 0;
    uint32_t width = 128;
    uint32_t height = 128;
    uint64_t stepIndex = 0;
    std::vector<uint8_t> rgbData{};
};

/// @brief Event posted to set an articulated joint position or velocity target.
struct AgentJointCommand {
    std::string jointName{};
    float targetPosition = 0.0f;
    float targetVelocity = 0.0f;
};

/// @brief Telemetry event posted when a sensor samples observations.
struct SensorSampleEvent {
    std::string sensorName{};
    std::string sensorType{};
    uint64_t stepIndex = 0;
    std::vector<float> observations{};
};

/// @brief Telemetry event posted when an actuator applies an action vector.
struct ActuatorAppliedEvent {
    std::string actuatorName{};
    std::string actuatorType{};
    uint64_t stepIndex = 0;
    std::vector<float> actions{};
};

/// @brief Combined default variant list of core Corium events, UI events, and Simulation events.
using DefaultSimEvents = std::variant<
    corium::QuitEvent,
    corium::TickEvent,
    corium::UpdateEvent,
    corium::ErrorEvent,
    corium::SignalEvent,
    corium::ui::WindowResizeEvent,
    corium::ui::WindowMoveEvent,
    corium::ui::FramebufferResizeEvent,
    corium::ui::WindowMinimizeEvent,
    corium::ui::WindowMaximizeEvent,
    corium::ui::WindowFocusEvent,
    corium::ui::WindowRefreshEvent,
    corium::ui::WindowContentScaleEvent,
    corium::ui::WindowCloseEvent,
    corium::ui::MouseMoveEvent,
    corium::ui::MouseButtonEvent,
    corium::ui::MouseScrollEvent,
    corium::ui::MouseEnterEvent,
    corium::ui::KeyEvent,
    corium::ui::CharEvent,
    MeshLoadEvent,
    CameraCommandEvent,
    CameraUpdateEvent,
    SimStepEvent,
    SimResetEvent,
    SenseTriggerEvent,
    AgentPerceptionEvent,
    AgentActionCommand,
    AgentObservationEvent,
    SensorFrameEvent,
    AgentJointCommand,
    SensorSampleEvent,
    ActuatorAppliedEvent,
    corium::logging::LogEvent
>;

} // namespace corium_sim
