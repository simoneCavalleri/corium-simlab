#include "corium_sim/physics/PhysicsEngine.hpp"
#include <algorithm>
#include <cmath>

namespace corium_sim::physics {

using namespace math;

void PhysicsEngine::step(scene::SimScene& scene, float deltaTime) noexcept
{
    if (deltaTime <= 0.0f) return;

    for (auto& entity : scene.entities()) {
        if (!entity.hasPhysics || entity.isStatic) continue;

        // 1. Apply gravity
        entity.velocity += _gravity * deltaTime;

        // 2. Integrate position & rotation
        entity.position += entity.velocity * deltaTime;
        entity.rotation += entity.angularVelocity * deltaTime;

        // 3. Apply linear & angular damping (friction)
        float lDampFactor = std::pow(_linearDamping, deltaTime * 60.0f);
        float aDampFactor = std::pow(_angularDamping, deltaTime * 60.0f);

        entity.velocity *= lDampFactor;
        entity.angularVelocity *= aDampFactor;

        // 4. Ground plane collision resolution
        solveGroundPlaneCollision(entity);
    }

    // 5. Solve entity vs entity AABB collisions
    solveEntityCollisions(scene);
}

void PhysicsEngine::solveGroundPlaneCollision(scene::SimEntity& entity) noexcept
{
    renderer::BoundingBox wb = entity.worldBounds();
    if (wb.min.y < _groundY) {
        float penetration = _groundY - wb.min.y;
        entity.position.y += penetration;

        // Stop downward velocity & apply ground friction
        if (entity.velocity.y < 0.0f) {
            entity.velocity.y = 0.0f;
        }
        entity.velocity.x *= 0.9f;
        entity.velocity.z *= 0.9f;
    }
}

void PhysicsEngine::solveEntityCollisions(scene::SimScene& scene) noexcept
{
    auto& entities = scene.entities();
    std::size_t count = entities.size();

    for (std::size_t i = 0; i < count; ++i) {
        if (!entities[i].hasPhysics || entities[i].name == "ground_grid") continue;

        for (std::size_t j = i + 1; j < count; ++j) {
            if (!entities[j].hasPhysics || entities[j].name == "ground_grid") continue;

            renderer::BoundingBox boxA = entities[i].worldBounds();
            renderer::BoundingBox boxB = entities[j].worldBounds();

            if (checkAABBCollision(boxA, boxB)) {
                // Elastic / impulse bounce response
                if (!entities[i].isStatic && !entities[j].isStatic) {
                    Vec3 pushDir = (entities[i].position - entities[j].position).normalized();
                    if (pushDir.lengthSq() < 0.001f) pushDir = Vec3{1.0f, 0.0f, 0.0f};

                    entities[i].position += pushDir * 0.05f;
                    entities[j].position -= pushDir * 0.05f;

                    // Bounce velocity swap
                    std::swap(entities[i].velocity, entities[j].velocity);
                } else if (!entities[i].isStatic) {
                    Vec3 pushDir = (entities[i].position - entities[j].position).normalized();
                    entities[i].position += pushDir * 0.1f;
                    entities[i].velocity = -entities[i].velocity * 0.5f;
                } else if (!entities[j].isStatic) {
                    Vec3 pushDir = (entities[j].position - entities[i].position).normalized();
                    entities[j].position += pushDir * 0.1f;
                    entities[j].velocity = -entities[j].velocity * 0.5f;
                }
            }
        }
    }
}

bool PhysicsEngine::checkAABBCollision(const renderer::BoundingBox& a, const renderer::BoundingBox& b) noexcept
{
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
           (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
           (a.min.z <= b.max.z && a.max.z >= b.min.z);
}

} // namespace corium_sim::physics
