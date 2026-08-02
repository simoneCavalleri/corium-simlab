#pragma once

#include <cstdint>
#include "corium_sim/math/Math.hpp"
#include "corium_sim/renderer/Camera.hpp"

namespace corium_sim::renderer {

/// @brief Onboard Agent Visual Sensor Camera (RGB & Depth Observation Camera).
class SensorCamera {
public:
    SensorCamera(uint32_t width = 128, uint32_t height = 128, float fovDegrees = 75.0f);
    ~SensorCamera() = default;

    /// @brief Set sensor resolution (e.g. 64x64, 128x128, 256x256).
    void setResolution(uint32_t width, uint32_t height) noexcept;

    /// @brief Set sensor Field of View in degrees.
    void setFov(float fovDegrees) noexcept;

    /// @brief Mount sensor camera on agent entity position with given orientation angles.
    void updateMountPose(
        const math::Vec3& agentPosition,
        const math::Vec3& agentRotation,
        const math::Vec3& mountOffset = {0.0f, 0.5f, 0.0f}
    ) noexcept;

    [[nodiscard]] uint32_t width() const noexcept { return _width; }
    [[nodiscard]] uint32_t height() const noexcept { return _height; }
    [[nodiscard]] const Camera& camera() const noexcept { return _camera; }

private:
    Camera _camera{};
    uint32_t _width = 128;
    uint32_t _height = 128;
    float _fov = 75.0f;
};

} // namespace corium_sim::renderer
