#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "corium_sim/agent/Concepts.hpp"
#include "corium_sim/math/Math.hpp"
#include "corium_sim/physics/Raycast.hpp"
#include "corium_sim/renderer/SensorCamera.hpp"
#include "corium_sim/renderer/WebGpuBackend.hpp"
#include "corium_sim/scene/SimEntity.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::agent::sensors {

/// @brief Onboard Agent Visual Camera Sensor with WebGPU Offscreen Rendering.
/// Produces an observation buffer containing normalized RGBA color values [Width * Height * 4].
template <uint32_t Width = 128, uint32_t Height = 128>
class GpuCameraSensor {
public:
    static constexpr std::size_t observation_size = Width * Height * 4;

    GpuCameraSensor(float fovDegrees = 75.0f, math::Vec3 mountOffset = {0.0f, 0.5f, 0.0f})
        : _fov(fovDegrees), _mountOffset(mountOffset) {}

    [[nodiscard]] std::span<const float> sample(
        const scene::SimEntity& entity,
        const scene::SimScene& scene,
        renderer::WebGpuBackend* gpuBackend = nullptr
    ) noexcept
    {
        if (!gpuBackend || !gpuBackend->isInitialized()) {
            _buffer.fill(0.0f);
            return std::span<const float>(_buffer.data(), observation_size);
        }

        if (!_target.isValid) {
            _target = gpuBackend->createOffscreenTarget(Width, Height);
        }

        _sensorCamera.setResolution(Width, Height);
        _sensorCamera.setFov(_fov);
        _sensorCamera.updateMountPose(entity.position, entity.rotation, _mountOffset);

        gpuBackend->renderOffscreen(_target, _sensorCamera.camera(), scene);

        auto pixels = gpuBackend->readOffscreenPixels(_target);
        if (pixels.size() == observation_size) {
            for (std::size_t i = 0; i < observation_size; ++i) {
                _buffer[i] = static_cast<float>(pixels[i]) / 255.0f;
            }
        } else {
            _buffer.fill(0.0f);
        }

        return std::span<const float>(_buffer.data(), observation_size);
    }

private:
    float _fov = 75.0f;
    math::Vec3 _mountOffset{0.0f, 0.5f, 0.0f};
    renderer::SensorCamera _sensorCamera{Width, Height, 75.0f};
    renderer::OffscreenTarget _target{};
    std::array<float, observation_size> _buffer{};
};

/// @brief GPU-Accelerated 3D LiDAR Sensor (with seamless CPU raycast fallback).
template <std::size_t RayCount = 180>
class GpuRaycastLidarSensor {
public:
    static constexpr std::size_t observation_size = RayCount;

    float maxDistance = 20.0f;
    float fovDegrees = 180.0f;
    math::Vec3 mountOffset{0.0f, 0.5f, 0.0f};
    bool enableVisualization = true;

    GpuRaycastLidarSensor(float maxDist = 20.0f, float fovDeg = 180.0f, bool viz = true, math::Vec3 offset = {0.0f, 0.5f, 0.0f})
        : maxDistance(maxDist), fovDegrees(fovDeg), mountOffset(offset), enableVisualization(viz),
          _sensorCamera(static_cast<uint32_t>(RayCount), 1, fovDeg) {}

    [[nodiscard]] const std::vector<math::Vec3>& hitPoints() const noexcept { return _hitPoints; }

    [[nodiscard]] std::span<const float> sample(
        const scene::SimEntity& entity,
        const scene::SimScene& scene,
        renderer::WebGpuBackend* gpuBackend = nullptr
    ) noexcept
    {
        _hitPoints.resize(RayCount);
        math::Vec3 origin = entity.position + mountOffset;
        float startAngle = -fovDegrees * 0.5f * math::DEG2RAD;
        float angleStep = (RayCount > 1) ? (fovDegrees * math::DEG2RAD / static_cast<float>(RayCount)) : 0.0f;
        float yawRad = entity.rotation.y * math::DEG2RAD;

        // Use GPU offscreen depth pass when GPU backend is available
        if (gpuBackend && gpuBackend->isInitialized()) {
            if (!_target.isValid) {
                _target = gpuBackend->createOffscreenTarget(static_cast<uint32_t>(RayCount), 1);
            }

            _sensorCamera.setResolution(static_cast<uint32_t>(RayCount), 1);
            _sensorCamera.setFov(fovDegrees);
            _sensorCamera.updateMountPose(entity.position, entity.rotation, mountOffset);

            gpuBackend->renderOffscreen(_target, _sensorCamera.camera(), scene);

            auto pixels = gpuBackend->readOffscreenPixels(_target);
            if (pixels.size() >= RayCount * 4) {
                for (std::size_t i = 0; i < RayCount; ++i) {
                    float r = static_cast<float>(pixels[i * 4]) / 255.0f;
                    float dist = r * maxDistance;
                    _distances[i] = dist;

                    float rayAngle = yawRad + startAngle + static_cast<float>(i) * angleStep;
                    math::Vec3 dir{std::sin(rayAngle), 0.0f, std::cos(rayAngle)};
                    _hitPoints[i] = origin + dir * dist;
                }
                updateSceneVisualization(scene);
                return std::span<const float>(_distances.data(), RayCount);
            }
        }

        // CPU Raycast fallback if GPU backend is null or uninitialized
        for (std::size_t i = 0; i < RayCount; ++i) {
            float rayAngle = yawRad + startAngle + static_cast<float>(i) * angleStep;
            math::Vec3 dir{std::sin(rayAngle), 0.0f, std::cos(rayAngle)};

            auto hit = physics::Raycast::castRay(scene, origin, dir, maxDistance);
            float dist = hit.hit ? hit.distance : maxDistance;
            _distances[i] = dist;
            _hitPoints[i] = origin + dir * dist;
        }

        updateSceneVisualization(scene);
        return std::span<const float>(_distances.data(), RayCount);
    }

private:
    void updateSceneVisualization(const scene::SimScene& scene) noexcept
    {
        if (!enableVisualization) return;
        auto& mutableScene = const_cast<scene::SimScene&>(scene);
        std::size_t stride = std::max<std::size_t>(1, RayCount / 45); // Limit density for smooth performance

        for (std::size_t i = 0; i < RayCount; i += stride) {
            std::string markerName = "_lidar_hit_" + std::to_string(i);
            if (auto* marker = mutableScene.findEntity(markerName)) {
                marker->position = _hitPoints[i];
            } else {
                mutableScene.addEntity(scene::SimEntity{
                    .name = markerName,
                    .material = renderer::Material::Metallic({0.1f, 0.95f, 0.2f, 1.0f}, 0.1f),
                    .position = _hitPoints[i],
                    .scale = {0.12f, 0.12f, 0.12f},
                    .isStatic = false
                });
            }
        }
    }

    renderer::SensorCamera _sensorCamera{static_cast<uint32_t>(RayCount), 1, 180.0f};
    renderer::OffscreenTarget _target{};
    std::array<float, RayCount> _distances{};
    std::vector<math::Vec3> _hitPoints{};
};

} // namespace corium_sim::agent::sensors
