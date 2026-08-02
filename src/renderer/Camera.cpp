#include "corium_sim/renderer/Camera.hpp"
#include <algorithm>
#include <cmath>

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
    _distance = std::clamp(distance, 0.5f, 300.0f);
    updateMatrices();
}

void Camera::setEyeAndTarget(const Vec3& eye, const Vec3& target) noexcept
{
    _target = target;
    Vec3 dir = eye - target;
    _distance = dir.length();
    if (_distance < 0.001f) _distance = 0.001f;

    dir = dir.normalized();
    _pitch = std::asin(std::clamp(dir.y, -1.0f, 1.0f)) * RAD2DEG;
    _yaw = std::atan2(dir.x, dir.z) * RAD2DEG;

    updateMatrices();
}

void Camera::focusOnBounds(const Vec3& minBounds, const Vec3& maxBounds) noexcept
{
    Vec3 center = (minBounds + maxBounds) * 0.5f;
    Vec3 size = maxBounds - minBounds;
    float maxExtent = std::max({size.x, size.y, size.z});

    _target = center;
    float fovRad = _fov * DEG2RAD;
    _distance = (maxExtent * 0.5f) / std::sin(fovRad * 0.5f) * 1.5f;
    if (_distance < 2.0f) _distance = 2.0f;

    updateMatrices();
}

void Camera::onMouseButton(int button, bool pressed, float x, float y, bool shiftPressed) noexcept
{
    _lastMouseX = x;
    _lastMouseY = y;

    if (pressed) {
        if (button == 0 && !shiftPressed) { // Left Click -> Orbit
            _isOrbiting = true;
            _isPanning = false;
        } else if (button == 1 || button == 2 || (button == 0 && shiftPressed)) { // Right or Middle Click or Shift+LMB -> Pan
            _isPanning = true;
            _isOrbiting = false;
        }
    } else {
        _isOrbiting = false;
        _isPanning = false;
    }
}

void Camera::onMouseMove(float x, float y) noexcept
{
    float deltaX = x - _lastMouseX;
    float deltaY = y - _lastMouseY;

    if (_isOrbiting) {
        _yaw += deltaX * 0.3f;
        _pitch += deltaY * 0.3f;
        _pitch = std::clamp(_pitch, -89.0f, 89.0f);
        updateMatrices();
    } else if (_isPanning) {
        Vec3 right = getRightVector();
        Vec3 up = getUpVector();

        float panSpeed = _distance * 0.0015f;
        _target = _target - (right * (deltaX * panSpeed)) + (up * (deltaY * panSpeed));
        updateMatrices();
    }

    _lastMouseX = x;
    _lastMouseY = y;
}

void Camera::onScroll(float xOffset, float yOffset) noexcept
{
    (void)xOffset;
    _distance -= yOffset * (_distance * 0.1f);
    _distance = std::clamp(_distance, 0.5f, 300.0f);
    updateMatrices();
}

void Camera::onKey(int key, bool pressed) noexcept
{
    // GLFW key codes: W=87, A=65, S=83, D=68, Q=81, E=69
    switch (key) {
        case 87: case 265: _keyW = pressed; break; // W or Up Arrow
        case 83: case 264: _keyS = pressed; break; // S or Down Arrow
        case 65: case 263: _keyA = pressed; break; // A or Left Arrow
        case 68: case 262: _keyD = pressed; break; // D or Right Arrow
        case 81: _keyQ = pressed; break;           // Q (Down)
        case 69: _keyE = pressed; break;           // E (Up)
        default: break;
    }
}

void Camera::update(float deltaTime) noexcept
{
    if (!_keyW && !_keyS && !_keyA && !_keyD && !_keyQ && !_keyE) return;

    Vec3 forward = getForwardVector();
    Vec3 right = getRightVector();
    Vec3 up{0.0f, 1.0f, 0.0f};

    // Flatten forward vector for ground-level camera navigation
    Vec3 groundForward{forward.x, 0.0f, forward.z};
    if (groundForward.lengthSq() > 0.001f) groundForward = groundForward.normalized();

    float moveSpeed = _distance * 1.5f * deltaTime;
    Vec3 moveDelta{0.0f, 0.0f, 0.0f};

    if (_keyW) moveDelta = moveDelta + groundForward * moveSpeed;
    if (_keyS) moveDelta = moveDelta - groundForward * moveSpeed;
    if (_keyD) moveDelta = moveDelta + right * moveSpeed;
    if (_keyA) moveDelta = moveDelta - right * moveSpeed;
    if (_keyE) moveDelta = moveDelta + up * moveSpeed;
    if (_keyQ) moveDelta = moveDelta - up * moveSpeed;

    _target = _target + moveDelta;
    updateMatrices();
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

Vec3 Camera::getForwardVector() const noexcept
{
    return (_target - getPosition()).normalized();
}

Vec3 Camera::getRightVector() const noexcept
{
    return Vec3::cross(getForwardVector(), Vec3{0.0f, 1.0f, 0.0f}).normalized();
}

Vec3 Camera::getUpVector() const noexcept
{
    return Vec3::cross(getRightVector(), getForwardVector()).normalized();
}

void Camera::updateMatrices() noexcept
{
    Vec3 eye = getPosition();
    Vec3 up{0.0f, 1.0f, 0.0f};

    _viewMatrix = Mat4::lookAt(eye, _target, up);
    _projMatrix = Mat4::perspective(_fov * DEG2RAD, _aspectRatio, _nearClip, _farClip);
}

} // namespace corium_sim::renderer
