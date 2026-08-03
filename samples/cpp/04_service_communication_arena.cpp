// =============================================================================
// Corium SimLab Sample #04 — Corium SimAgentApp & Service Architecture
// =============================================================================
//
// This sample demonstrates:
//   1. Defining an agent blueprint with existing makeAgentSpec() 7-Pillar syntax.
//   2. Automatic behind-the-scenes creation of a first-class Corium SimAgentApp.
//   3. Corium AgentRuntime initialization and 100% deterministic inter-service stepping.
// =============================================================================

#include <iostream>
#include <iomanip>
#include <cmath>
#include "corium_sim/SimLab.hpp"

using namespace corium_sim;
using namespace corium_sim::agent;
using namespace corium_sim::math;
using namespace corium_sim::scene;

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=========================================================================\n";
    std::cout << " Corium SimLab — Sample #04: Corium SimAgentApp & Service Architecture\n";
    std::cout << " Features: makeAgentSpec() -> Corium SimAgentApp generated behind the scenes\n";
    std::cout << "=========================================================================\n\n";

    // Track simulated state
    Vec3 agentPos{0.0f, 0.5f, 0.0f};
    Vec3 targetPos{4.0f, 0.75f, -2.0f};
    Vec3 agentVel{0.0f, 0.0f, 0.0f};

    // 1. DEFINE REUSABLE AGENT SPECIFICATION BLUEPRINT (Existing 7-Pillar Syntax!)
    auto amrAgentSpec = makeAgentSpec()
        .withModel(SimEntity{ .name = "amr_robot", .position = {0.0f, 0.5f, 0.0f}, .mass = 15.0f })
        .withSensors(sensors::ImuSensor{})
        .withPerceptionChain<6>([&](const auto& rawObs) {
            (void)rawObs;
            // Supply position and target coordinates
            return std::array<float, 6>{agentPos.x, agentPos.y, agentPos.z, targetPos.x, targetPos.y, targetPos.z};
        })
        .withActuators(actuators::DifferentialDriveActuator{})
        .withPolicy([&](const std::vector<float>& obs) -> std::vector<float> {
            if (obs.size() < 6) return {0.0f, 0.0f};
            float dx = obs[3] - obs[0];
            float dz = obs[5] - obs[2];
            float dist = std::sqrt(dx * dx + dz * dz);
            float desiredYaw = std::atan2(dx, dz) * RAD2DEG;
            return {std::clamp(dist * 0.5f, 0.2f, 1.5f), desiredYaw};
        });

    // 2. GENERATE CORIUM AGENT APP BOUND TO AGENT INSTANCE
    using AgentType = typename decltype(amrAgentSpec)::AgentType;
    auto agentPtr = std::make_shared<AgentType>(
        std::move(amrAgentSpec.model()),
        std::move(amrAgentSpec.perception()),
        std::move(amrAgentSpec.actuators())
    );
    auto agentApp = amrAgentSpec.createApp(agentPtr);

    // 3. INITIALIZE CORIUM AGENT RUNTIME
    AgentRuntime runtime;
    runtime.initialize(*agentApp);

    // 4. EXECUTE 100% DETERMINISTIC INTER-SERVICE TIMESTEPS
    std::cout << "[Step Loop] Executing 10 Deterministic Service-to-Service Timesteps...\n";

    for (uint64_t step = 1; step <= 10; ++step) {
        agentApp->step(step, 0.01667f);

        // Update body position from actuator velocity output
        agentPtr->body().position = agentPtr->body().position + agentPtr->body().velocity * 0.01667f;
        agentPos = agentPtr->body().position;

        std::cout << "  Step #" << std::setw(2) << step
                  << " | Agent Pos: (" << std::fixed << std::setprecision(3)
                  << agentPos.x << ", " << agentPos.y << ", " << agentPos.z << ")"
                  << " | Dist to Target: " << (agentPos - targetPos).length() << "m\n";
    }

    std::cout << "\n[Trace Verification] Summary of Recorded Inter-Service Events:\n";
    agentApp->tracer().logSummary();

    std::cout << "\n[Success] 100% Deterministic SimAgentApp Execution Complete.\n";
    return 0;
}
