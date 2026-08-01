#include "corium_sim/renderer/Camera.hpp"
#include <algorithm>

namespace corium_sim::renderer {

using namespace math;

Camera::Camera()
{
    updateMatrices();
}

void Camera::setViewportSize(uint32_t width, uint32_t height) noexcept
{
    if (height > 0) {
        _aspectRatio = static_cast<float>(width) / static_cast<float>(height);
        updateMatrices();
    }
}

void Camera::setFov(float fovDegrees) noexcept
{
    _fov = std::clamp(fovDegrees, 10.0f, 120.0f);
    updateMatrices();
}

void Camera::setClipping(float nearPlane, float farPlane) noexcept
{
    _nearClip = nearPlane;
    _farClip = farPlane;
    updateMatrices();
}

void Camera::setTarget(const Vec3& target) noexcept
{
    _target = target;
    updateMatrices();
}

void Camera::setDistance(float distance) noexcept
{
    _distance = std::clamp(distance, 1.0f, 100.0f);
    updateMatrices();
}

void Camera::onMouseButton(int button, bool pressed, float x, float y) noexcept
{
    if (button == 0) { // Left Mouse Button
        _isDragging = pressed;
        _lastMouseX = x;
        _lastMouseY = y;
    }
}

void Camera::onMouseMove(float x, float y) noexcept
{
    if (_isDragging) {
        float deltaX = x - _lastMouseX;
        float deltaY = y - _lastMouseY;

        _yaw += deltaX * 0.3f;
        _pitch += deltaY * 0.3f;
        _pitch = std::clamp(_pitch, -89.0f, 89.0f);

        updateMatrices();
    }
    _lastMouseX = x;
    _lastMouseY = y;
}

void Camera::onScroll(float yOffset) noexcept
{
    _distance -= yOffset * 0.5f;
    _distance = std::clamp(_distance, 1.5f, 50.0f);
    updateMatrices();
}

void Camera::onKey(int key, bool pressed) noexcept
{
    (void)key;
    (void)pressed;
}

void Camera::update(float deltaTime) noexcept
{
    (void)deltaTime;
    // Smooth transitions if needed
}

Vec3 Camera::getPosition() const noexcept
{
    float yawRad = _yaw * DEG2RAD;
    float pitchRad = _pitch * DEG2RAD;

    float x = _target.x + _distance * std::cos(pitchRad) * std::sin(yawRad);
    float y = _target.y + _distance * std::sin(pitchRad);
    float z = _target.z + _distance * std::cos(pitchRad) * std::cos(yawRad);

    return Vec3{x, y, z};
}

void Camera::updateMatrices() noexcept
{
    Vec3 eye = getPosition();
    Vec3 up{0.0f, 1.0f, 0.0f};

    _viewMatrix = Mat4::lookAt(eye, _target, up);
    _projMatrix = Mat4::perspective(_fov * DEG2RAD, _aspectRatio, _nearClip, _farClip);
}

} // namespace corium_sim::renderer
