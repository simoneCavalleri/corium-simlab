#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <utility>

#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/agent/SensorSuite.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent {

/// @brief Perception Processing & Sensor Fusion Pipeline.
/// Takes raw sensor observations from a SensorSuite, executes user-defined processing
/// (Sensor Fusion, Segmentation, Kalman filtering, or Feature Extraction), and produces
/// a refined fused observation vector for the agent policy.
/// @tparam SensorSuiteType Underling variadic SensorSuite.
/// @tparam OutputSize Compile-time size of the processed/fused observation payload.
/// @tparam ProcessorFunc User function with signature `std::array<float, OutputSize>(const RawObservationBuffer&)`.
template <typename SensorSuiteType, std::size_t OutputSize, typename ProcessorFunc>
class PerceptionPipeline {
public:
    using RawObservationBuffer = typename SensorSuiteType::ObservationBuffer;
    using ProcessedObservationBuffer = std::array<float, OutputSize>;
    using ObservationBuffer = ProcessedObservationBuffer;

    static constexpr std::size_t total_observation_size = OutputSize;
    static constexpr std::size_t observation_size = OutputSize;

    PerceptionPipeline() = default;

    PerceptionPipeline(SensorSuiteType sensorSuite, ProcessorFunc processor)
        : _sensorSuite(std::move(sensorSuite)), _processor(std::move(processor)) {}

    /// @brief Access underlying sensor suite (mutable).
    [[nodiscard]] inline SensorSuiteType& sensorSuite() noexcept { return _sensorSuite; }

    /// @brief Access underlying sensor suite (const).
    [[nodiscard]] inline const SensorSuiteType& sensorSuite() const noexcept { return _sensorSuite; }

    /// @brief Sample raw sensors in environment scene and run user-defined perception processing / sensor fusion.
    [[nodiscard]] inline ProcessedObservationBuffer observe(const scene::SimEntity& entity, const scene::SimScene& scene) noexcept
    {
        RawObservationBuffer rawObs = _sensorSuite.observe(entity, scene);
        return _processor(rawObs);
    }

private:
    SensorSuiteType _sensorSuite{};
    ProcessorFunc _processor{};
};

/// @brief Helper function for building a PerceptionPipeline with template argument deduction.
template <std::size_t OutputSize, typename SensorSuiteType, typename ProcessorFunc>
[[nodiscard]] constexpr auto makePerceptionPipeline(SensorSuiteType&& suite, ProcessorFunc&& processor)
{
    using SuiteType = std::decay_t<SensorSuiteType>;
    using ProcType  = std::decay_t<ProcessorFunc>;
    return PerceptionPipeline<SuiteType, OutputSize, ProcType>(
        std::forward<SensorSuiteType>(suite),
        std::forward<ProcessorFunc>(processor)
    );
}

} // namespace corium_sim::agent
