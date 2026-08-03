#pragma once

#include "corium_sim/math/Math.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::physics {

/// @brief Lightweight 3D Rigid Body Physics & Collision Engine for Real-Time Agent Simulation.
class PhysicsEngine {
public:
    PhysicsEngine() = default;
    ~PhysicsEngine() = default;

    /// @brief Enable or disable global gravity vector (default: {0, -9.81, 0}).
    void setGravity(const math::Vec3& gravity) noexcept { _gravity = gravity; }
    [[nodiscard]] math::Vec3 gravity() const noexcept { return _gravity; }

    /// @brief Set linear & angular damping friction coefficients (default: 0.90).
    void setDamping(float linearDamping, float angularDamping) noexcept
    {
        _linearDamping = linearDamping;
        _angularDamping = angularDamping;
    }

    /// @brief Advance physics simulation step for all active scene entities.
    void step(scene::SimScene& scene, float deltaTime) noexcept;

    /// @brief Reset physics engine internal state.
    void reset() noexcept {}


    /// @brief Check AABB overlap collision between two bounding boxes.
    [[nodiscard]] static bool checkAABBCollision(
        const renderer::BoundingBox& a,
        const renderer::BoundingBox& b
    ) noexcept;

private:
    void solveGroundPlaneCollision(scene::SimEntity& entity) noexcept;
    void solveEntityCollisions(scene::SimScene& scene) noexcept;

    math::Vec3 _gravity{0.0f, 0.0f, 0.0f}; // Zero gravity default for ground-plane agents
    float _linearDamping{0.85f};
    float _angularDamping{0.80f};
    float _groundY{0.0f};
};

} // namespace corium_sim::physics
