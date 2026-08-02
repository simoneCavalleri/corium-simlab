#pragma once

#include <cstdint>
#include "corium_sim/math/Math.hpp"

namespace corium_sim::renderer {

/// @brief Interactive 3D Orbit, Pan, and Agent Controller Camera.
class Camera {
public:
    Camera();

    void setViewportSize(uint32_t width, uint32_t height) noexcept;
    void setFov(float fovDegrees) noexcept;
    void setClipping(float nearPlane, float farPlane) noexcept;
    void setTarget(const math::Vec3& target) noexcept;
    void setDistance(float distance) noexcept;

    /// @brief Set explicit camera eye position and look-at target.
    void setEyeAndTarget(const math::Vec3& eye, const math::Vec3& target) noexcept;

    /// @brief Automatically position and focus camera to fit a 3D asset bounding box in view.
    void focusOnBounds(const math::Vec3& minBounds, const math::Vec3& maxBounds) noexcept;

    // Input Event Handlers
    void onMouseButton(int button, bool pressed, float x, float y, bool shiftPressed = false) noexcept;
    void onMouseMove(float x, float y) noexcept;
    void onScroll(float xOffset, float yOffset) noexcept;
    void onKey(int key, bool pressed) noexcept;

    /// @brief Update smooth camera keyboard panning & physics per frame.
    void update(float deltaTime) noexcept;

    [[nodiscard]] math::Vec3 getPosition() const noexcept;
    [[nodiscard]] math::Vec3 getTarget() const noexcept { return _target; }
    [[nodiscard]] math::Vec3 getForwardVector() const noexcept;
    [[nodiscard]] math::Vec3 getRightVector() const noexcept;
    [[nodiscard]] math::Vec3 getUpVector() const noexcept;

    [[nodiscard]] float fov() const noexcept { return _fov; }
    [[nodiscard]] float distance() const noexcept { return _distance; }
    [[nodiscard]] const math::Mat4& getViewMatrix() const noexcept { return _viewMatrix; }
    [[nodiscard]] const math::Mat4& getProjectionMatrix() const noexcept { return _projMatrix; }
    [[nodiscard]] math::Mat4 getViewProjectionMatrix() const noexcept { return _projMatrix * _viewMatrix; }

private:
    void updateMatrices() noexcept;

    math::Vec3 _target{0.0f, 1.0f, 0.0f};
    float _distance{8.0f};
    float _yaw{45.0f};   // Degrees
    float _pitch{25.0f}; // Degrees

    float _fov{60.0f}; // Degrees
    float _aspectRatio{16.0f / 9.0f};
    float _nearClip{0.1f};
    float _farClip{500.0f};

    // Interaction Modes
    bool _isOrbiting{false};
    bool _isPanning{false};
    float _lastMouseX{0.0f};
    float _lastMouseY{0.0f};

    // Key states for WASD/QE movement
    bool _keyW{false};
    bool _keyA{false};
    bool _keyS{false};
    bool _keyD{false};
    bool _keyQ{false};
    bool _keyE{false};

    math::Mat4 _viewMatrix{math::Mat4::identity()};
    math::Mat4 _projMatrix{math::Mat4::identity()};
};

} // namespace corium_sim::renderer
