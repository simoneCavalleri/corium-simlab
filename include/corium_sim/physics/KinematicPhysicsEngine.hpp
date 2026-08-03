#pragma once

#include "corium_sim/kinematics/JointKinematics.hpp"
#include "corium_sim/physics/PhysicsConcept.hpp"
#include "corium_sim/scene/SimScene.hpp"

namespace corium_sim::physics {

/// @brief Lightweight Kinematic-Only Physics Engine for articulated joint propagation without rigid body ground friction.
class KinematicPhysicsEngine {
public:
    KinematicPhysicsEngine() = default;

    /// @brief Advance kinematic simulation step by integrating velocity into positions and updating joint transforms.
    void step(scene::SimScene& scene, float dt) noexcept
    {
        for (auto& entity : scene.entities()) {
            if (!entity.isStatic) {
                entity.position.x += entity.velocity.x * dt;
                entity.position.y += entity.velocity.y * dt;
                entity.position.z += entity.velocity.z * dt;

                entity.rotation.x += entity.angularVelocity.x * dt;
                entity.rotation.y += entity.angularVelocity.y * dt;
                entity.rotation.z += entity.angularVelocity.z * dt;
            }
        }
        _jointKinematics.updateKinematics(scene, dt);
    }

    /// @brief Reset internal kinematic state.
    void reset() noexcept {}

    [[nodiscard]] kinematics::JointKinematics& jointKinematics() noexcept { return _jointKinematics; }

private:
    kinematics::JointKinematics _jointKinematics{};
};

// Enforce C++20 PhysicsBackend concept
static_assert(PhysicsBackend<KinematicPhysicsEngine>, "KinematicPhysicsEngine must satisfy PhysicsBackend!");

} // namespace corium_sim::physics
