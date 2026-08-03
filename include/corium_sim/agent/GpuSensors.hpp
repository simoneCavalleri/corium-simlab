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

    [[nodiscard]] std::span<const float> sample(
        const scene::SimEntity& entity,
        const scene::SimScene& scene,
        renderer::WebGpuBackend* gpuBackend = nullptr
    ) noexcept
    {
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
                    _distances[i] = r * maxDistance;
                }
                return std::span<const float>(_distances.data(), RayCount);
            }
        }

        // CPU Raycast fallback if GPU backend is null or uninitialized
        math::Vec3 origin = entity.position + mountOffset;
        float startAngle = -fovDegrees * 0.5f * math::DEG2RAD;
        float angleStep = (RayCount > 1) ? (fovDegrees * math::DEG2RAD / static_cast<float>(RayCount)) : 0.0f;
        float yawRad = entity.rotation.y * math::DEG2RAD;

        for (std::size_t i = 0; i < RayCount; ++i) {
            float rayAngle = yawRad + startAngle + static_cast<float>(i) * angleStep;
            math::Vec3 dir{std::sin(rayAngle), 0.0f, std::cos(rayAngle)};

            auto hit = physics::Raycast::castRay(scene, origin, dir, maxDistance);
            _distances[i] = hit.hit ? hit.distance : maxDistance;
        }
        return std::span<const float>(_distances.data(), RayCount);
    }

private:
    renderer::SensorCamera _sensorCamera{static_cast<uint32_t>(RayCount), 1, 180.0f};
    renderer::OffscreenTarget _target{};
    std::array<float, RayCount> _distances{};
};

} // namespace corium_sim::agent::sensors
