#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <corium/IEventSink.hpp>
#include "corium_sim/events/SimEvents.hpp"
#include "corium_sim/Log.hpp"

namespace corium_sim::events {

/// @brief Structured trace entry recorded by SimEventTracer.
struct TraceEntry {
    uint64_t stepIndex = 0;
    double timestampMs = 0.0;
    std::string eventType{};
    std::string summary{};
    std::string detailsJson{};
};

/// @brief Event Tracer for recording, inspecting, and exporting Corium simulation events.
class SimEventTracer {
public:
    SimEventTracer() = default;

    /// @brief Enable event tracing.
    void startTracing() noexcept
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _tracingEnabled = true;
    }

    /// @brief Disable event tracing.
    void stopTracing() noexcept
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _tracingEnabled = false;
    }

    /// @brief Check if tracing is currently enabled.
    [[nodiscard]] bool isTracing() const noexcept
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _tracingEnabled;
    }

    /// @brief Clear recorded event trace history.
    void clear() noexcept
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _entries.clear();
        _startTime = std::chrono::steady_clock::now();
    }

    /// @brief Get total number of recorded trace entries.
    [[nodiscard]] std::size_t traceCount() const noexcept
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _entries.size();
    }

    /// @brief Get recorded trace entries copy.
    [[nodiscard]] std::vector<TraceEntry> traces() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _entries;
    }

    /// @brief Record a generic simulation trace entry manually or from handler.
    void record(uint64_t stepIndex, std::string eventType, std::string summary, std::string detailsJson = "{}")
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_tracingEnabled) return;

        double timestampMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - _startTime
        ).count();

        _entries.push_back(TraceEntry{
            .stepIndex = stepIndex,
            .timestampMs = timestampMs,
            .eventType = std::move(eventType),
            .summary = std::move(summary),
            .detailsJson = std::move(detailsJson)
        });
    }

    /// @brief Register handlers on Corium EventBus (or Runtime/App) to trace all simulation events.
    template <typename EventBusOrApp>
    void registerWith(EventBusOrApp& busOrApp)
    {
        auto reg = [&](auto&& handler) {
            if constexpr (requires { busOrApp.registerHandler(std::forward<decltype(handler)>(handler)); }) {
                busOrApp.registerHandler(std::forward<decltype(handler)>(handler));
            } else if constexpr (requires { busOrApp.events().registerHandler(std::forward<decltype(handler)>(handler)); }) {
                busOrApp.events().registerHandler(std::forward<decltype(handler)>(handler));
            } else if constexpr (requires { busOrApp.on(std::forward<decltype(handler)>(handler)); }) {
                busOrApp.on(std::forward<decltype(handler)>(handler));
            }
        };

        // 1. SimStepEvent
        reg([this](const SimStepEvent& evt) {
            std::ostringstream ss;
            ss << "Step #" << evt.stepIndex << " (dt: " << std::fixed << std::setprecision(4) << evt.deltaTime << "s)";
            std::ostringstream json;
            json << "{\"stepIndex\":" << evt.stepIndex << ",\"deltaTime\":" << evt.deltaTime << "}";
            record(evt.stepIndex, "SimStepEvent", ss.str(), json.str());
        });

        // 2. SimResetEvent
        reg([this](const SimResetEvent& evt) {
            std::ostringstream ss;
            ss << "Reset Episode #" << evt.episodeIndex << " (seed: " << evt.seed << ")";
            std::ostringstream json;
            json << "{\"episodeIndex\":" << evt.episodeIndex << ",\"seed\":" << evt.seed << "}";
            record(0, "SimResetEvent", ss.str(), json.str());
        });

        // 2b. SenseTriggerEvent
        reg([this](const SenseTriggerEvent& evt) {
            std::ostringstream ss;
            ss << "Sense Trigger for Agent #" << evt.agentId << " Step #" << evt.stepIndex;
            std::ostringstream json;
            json << "{\"agentId\":" << evt.agentId << ",\"stepIndex\":" << evt.stepIndex << "}";
            record(evt.stepIndex, "SenseTriggerEvent", ss.str(), json.str());
        });

        // 2c. AgentPerceptionEvent
        reg([this](const AgentPerceptionEvent& evt) {
            std::ostringstream ss;
            ss << "Perception Event for Agent #" << evt.agentId << " Step #" << evt.stepIndex << " (" << evt.observations.size() << " floats)";
            std::ostringstream json;
            json << "{\"agentId\":" << evt.agentId << ",\"stepIndex\":" << evt.stepIndex << ",\"count\":" << evt.observations.size() << "}";
            record(evt.stepIndex, "AgentPerceptionEvent", ss.str(), json.str());
        });

        // 3. AgentActionCommand
        reg([this](const AgentActionCommand& evt) {
            std::ostringstream ss;
            ss << "Agent #" << evt.agentId << " Action: forward=" << evt.moveForward << ", yaw=" << evt.turnYaw << ", up=" << evt.moveUp;
            std::ostringstream json;
            json << "{\"agentId\":" << evt.agentId << ",\"moveForward\":" << evt.moveForward
                 << ",\"turnYaw\":" << evt.turnYaw << ",\"moveUp\":" << evt.moveUp << "}";
            record(0, "AgentActionCommand", ss.str(), json.str());
        });

        // 4. AgentObservationEvent
        reg([this](const AgentObservationEvent& evt) {
            std::ostringstream ss;
            ss << "Agent #" << evt.agentId << " Step #" << evt.stepIndex
               << " Pos=(" << std::fixed << std::setprecision(2) << evt.posX << "," << evt.posY << "," << evt.posZ << ")"
               << " Reward=" << evt.reward;
            std::ostringstream json;
            json << "{\"agentId\":" << evt.agentId << ",\"stepIndex\":" << evt.stepIndex
                 << ",\"posX\":" << evt.posX << ",\"posY\":" << evt.posY << ",\"posZ\":" << evt.posZ
                 << ",\"reward\":" << evt.reward << ",\"isTerminated\":" << (evt.isTerminated ? "true" : "false") << "}";
            record(evt.stepIndex, "AgentObservationEvent", ss.str(), json.str());
        });

        // 5. SensorSampleEvent
        reg([this](const SensorSampleEvent& evt) {
            std::ostringstream ss;
            ss << "Sensor '" << evt.sensorName << "' (" << evt.sensorType << ") Step #" << evt.stepIndex
               << " -> " << evt.observations.size() << " floats sampled";
            std::ostringstream json;
            json << "{\"sensorName\":\"" << evt.sensorName << "\",\"sensorType\":\"" << evt.sensorType
                 << "\",\"stepIndex\":" << evt.stepIndex << ",\"count\":" << evt.observations.size() << "}";
            record(evt.stepIndex, "SensorSampleEvent", ss.str(), json.str());
        });

        // 6. ActuatorAppliedEvent
        reg([this](const ActuatorAppliedEvent& evt) {
            std::ostringstream ss;
            ss << "Actuator '" << evt.actuatorName << "' (" << evt.actuatorType << ") Step #" << evt.stepIndex
               << " -> " << evt.actions.size() << " actions applied";
            std::ostringstream json;
            json << "{\"actuatorName\":\"" << evt.actuatorName << "\",\"actuatorType\":\"" << evt.actuatorType
                 << "\",\"stepIndex\":" << evt.stepIndex << ",\"count\":" << evt.actions.size() << "}";
            record(evt.stepIndex, "ActuatorAppliedEvent", ss.str(), json.str());
        });

        // 7. SensorFrameEvent
        reg([this](const SensorFrameEvent& evt) {
            std::ostringstream ss;
            ss << "Visual Sensor #" << evt.sensorId << " Frame (" << evt.width << "x" << evt.height << ") Step #" << evt.stepIndex;
            std::ostringstream json;
            json << "{\"sensorId\":" << evt.sensorId << ",\"width\":" << evt.width << ",\"height\":" << evt.height
                 << ",\"stepIndex\":" << evt.stepIndex << ",\"bytes\":" << evt.rgbData.size() << "}";
            record(evt.stepIndex, "SensorFrameEvent", ss.str(), json.str());
        });

        // 8. AgentJointCommand
        reg([this](const AgentJointCommand& evt) {
            std::ostringstream ss;
            ss << "Joint '" << evt.jointName << "' targetPos=" << evt.targetPosition << ", targetVel=" << evt.targetVelocity;
            std::ostringstream json;
            json << "{\"jointName\":\"" << evt.jointName << "\",\"targetPosition\":" << evt.targetPosition
                 << ",\"targetVelocity\":" << evt.targetVelocity << "}";
            record(0, "AgentJointCommand", ss.str(), json.str());
        });
    }

    /// @brief Output trace summary to stdout / logging console.
    void logSummary() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        CORIUM_LOG_INFO("SimEventTracer", "=== Corium Event Trace Summary (Total Entries: ", _entries.size(), ") ===");
        for (std::size_t i = 0; i < _entries.size() && i < 100; ++i) {
            const auto& e = _entries[i];
            CORIUM_LOG_INFO("SimEventTracer", "[", std::fixed, std::setprecision(2), e.timestampMs, "ms] [", e.eventType, "] ", e.summary);
        }
        if (_entries.size() > 100) {
            CORIUM_LOG_INFO("SimEventTracer", "... (", _entries.size() - 100, " more entries truncated)");
        }
    }

    /// @brief Export event trace log as JSON file.
    bool exportToJson(const std::string& filePath) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        std::ofstream file(filePath);
        if (!file.is_open()) {
            CORIUM_LOG_ERROR("SimEventTracer", "Failed to open output trace file: ", filePath);
            return false;
        }

        file << "[\n";
        for (std::size_t i = 0; i < _entries.size(); ++i) {
            const auto& e = _entries[i];
            file << "  {\n"
                 << "    \"index\": " << i << ",\n"
                 << "    \"stepIndex\": " << e.stepIndex << ",\n"
                 << "    \"timestampMs\": " << e.timestampMs << ",\n"
                 << "    \"eventType\": \"" << e.eventType << "\",\n"
                 << "    \"summary\": \"" << e.summary << "\",\n"
                 << "    \"details\": " << e.detailsJson << "\n"
                 << "  }" << (i + 1 < _entries.size() ? "," : "") << "\n";
        }
        file << "]\n";
        CORIUM_LOG_INFO("SimEventTracer", "Successfully exported ", _entries.size(), " event traces to: ", filePath);
        return true;
    }

private:
    mutable std::mutex _mutex{};
    bool _tracingEnabled = true;
    std::chrono::steady_clock::time_point _startTime = std::chrono::steady_clock::now();
    std::vector<TraceEntry> _entries{};
};

} // namespace corium_sim::events
