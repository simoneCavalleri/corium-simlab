#include "corium_sim/renderer/SensorCamera.hpp"
#include <cmath>

namespace corium_sim::renderer {

using namespace math;

SensorCamera::SensorCamera(uint32_t width, uint32_t height, float fovDegrees)
    : _width(width), _height(height), _fov(fovDegrees)
{
    _camera.setViewportSize(_width, _height);
    _camera.setFov(_fov);
}

void SensorCamera::setResolution(uint32_t width, uint32_t height) noexcept
{
    _width = width;
    _height = height;
    _camera.setViewportSize(_width, _height);
}

void SensorCamera::setFov(float fovDegrees) noexcept
{
    _fov = fovDegrees;
    _camera.setFov(_fov);
}

void SensorCamera::updateMountPose(
    const Vec3& agentPosition,
    const Vec3& agentRotation,
    const Vec3& mountOffset
) noexcept
{
    float yawRad = agentRotation.y * DEG2RAD;
    float pitchRad = agentRotation.x * DEG2RAD;

    // Forward direction vector
    Vec3 forward{
        std::sin(yawRad) * std::cos(pitchRad),
        std::sin(-pitchRad),
        std::cos(yawRad) * std::cos(pitchRad)
    };

    Vec3 eye = agentPosition + mountOffset;
    Vec3 target = eye + forward * 5.0f;

    _camera.setEyeAndTarget(eye, target);
}

} // namespace corium_sim::renderer
