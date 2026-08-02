#include "corium_sim/kinematics/JointKinematics.hpp"
#include <algorithm>
#include <cmath>

namespace corium_sim::kinematics {

using namespace math;

void JointKinematics::updateKinematics(scene::SimScene& scene, float deltaTime) noexcept
{
    auto& joints = scene.joints();

    for (auto& joint : joints) {
        // Integrate joint velocity targets
        if (deltaTime > 0.0f && std::abs(joint.targetVelocity) > 0.0001f) {
            joint.position += joint.targetVelocity * deltaTime;
        }

        // Enforce joint limits
        joint.position = std::clamp(joint.position, joint.minLimit, joint.maxLimit);

        scene::SimEntity* parent = scene.findEntity(joint.parentName);
        scene::SimEntity* child = scene.findEntity(joint.childName);

        if (!child) continue;
        child->hasPhysics = false;

        if (!parent) {
            // Root joint relative to origin
            if (joint.type == JointType::Revolute) {
                child->rotation = joint.axis * (joint.position * RAD2DEG);
                child->position = joint.anchor;
            } else if (joint.type == JointType::Prismatic) {
                child->position = joint.anchor + (joint.axis * joint.position);
            }
        } else {
            // Child link pose relative to parent
            if (joint.type == JointType::Revolute) {
                child->rotation = parent->rotation + (joint.axis * (joint.position * RAD2DEG));
                child->position = parent->position + joint.anchor;
            } else if (joint.type == JointType::Prismatic) {
                child->rotation = parent->rotation;
                child->position = parent->position + joint.anchor + (joint.axis * joint.position);
            } else if (joint.type == JointType::Fixed) {
                child->rotation = parent->rotation;
                child->position = parent->position + joint.anchor;
            }
        }
    }
}

bool JointKinematics::solveIK(
    scene::SimScene& scene,
    const std::string& endEffectorName,
    const Vec3& targetPosition,
    uint32_t maxIterations,
    float tolerance
) noexcept
{
    scene::SimEntity* endEffector = scene.findEntity(endEffectorName);
    if (!endEffector) return false;

    const float stepSize = 0.05f;

    for (uint32_t iter = 0; iter < maxIterations; ++iter) {
        updateKinematics(scene, 0.0f);
        Vec3 currentPos = endEffector->position;
        Vec3 error = targetPosition - currentPos;

        if (error.length() < tolerance) {
            return true;
        }

        for (auto& joint : scene.joints()) {
            float delta = 0.001f;
            float originalPos = joint.position;

            joint.position = originalPos + delta;
            updateKinematics(scene, 0.0f);
            Vec3 perturbedPos = endEffector->position;

            Vec3 gradient = (perturbedPos - currentPos) * (1.0f / delta);
            float dErr = error.x * gradient.x + error.y * gradient.y + error.z * gradient.z;

            joint.position = std::clamp(originalPos + stepSize * dErr, joint.minLimit, joint.maxLimit);
        }
    }

    updateKinematics(scene, 0.0f);
    return (endEffector->position - targetPosition).length() < tolerance;
}

} // namespace corium_sim::kinematics
