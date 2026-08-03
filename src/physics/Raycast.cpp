#include "corium_sim/physics/Raycast.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace corium_sim::physics {

using namespace math;

static bool intersectAABB(
    const Vec3& rayOrigin,
    const Vec3& rayDir,
    const Vec3& boxMin,
    const Vec3& boxMax,
    float maxDist,
    float& outT
) noexcept {
    float orig[3] = {rayOrigin.x, rayOrigin.y, rayOrigin.z};
    float dir[3] = {rayDir.x, rayDir.y, rayDir.z};
    float bMin[3] = {boxMin.x, boxMin.y, boxMin.z};
    float bMax[3] = {boxMax.x, boxMax.y, boxMax.z};

    float tmin = 0.0f;
    float tmax = maxDist;

    for (int i = 0; i < 3; ++i) {
        float d = std::abs(dir[i]) < 1e-6f ? (dir[i] >= 0.0f ? 1e-6f : -1e-6f) : dir[i];
        float invD = 1.0f / d;
        float t0 = (bMin[i] - orig[i]) * invD;
        float t1 = (bMax[i] - orig[i]) * invD;

        if (invD < 0.0f) std::swap(t0, t1);

        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);

        if (tmax <= tmin) return false;
    }

    outT = tmin;
    return true;
}

RaycastHit Raycast::castRay(
    const scene::SimScene& scene,
    const Vec3& origin,
    const Vec3& direction,
    float maxDistance
) noexcept {
    RaycastHit closestHit{};
    closestHit.hit = false;
    closestHit.distance = maxDistance;

    Vec3 dirNorm = direction.normalized();

    for (const auto& entity : scene.entities()) {
        Vec3 scaledMin{entity.localBounds.min.x * entity.scale.x, entity.localBounds.min.y * entity.scale.y, entity.localBounds.min.z * entity.scale.z};
        Vec3 scaledMax{entity.localBounds.max.x * entity.scale.x, entity.localBounds.max.y * entity.scale.y, entity.localBounds.max.z * entity.scale.z};

        Vec3 boxMin = entity.position + scaledMin;
        Vec3 boxMax = entity.position + scaledMax;

        float t = 0.0f;
        if (intersectAABB(origin, dirNorm, boxMin, boxMax, maxDistance, t)) {
            if (t > 0.001f && t < closestHit.distance) {
                closestHit.hit = true;
                closestHit.distance = t;
                closestHit.point = origin + (dirNorm * t);
                closestHit.entityName = entity.name;

                Vec3 center = (boxMin + boxMax) * 0.5f;
                closestHit.normal = (closestHit.point - center).normalized();
            }
        }
    }

    return closestHit;
}

std::vector<RaycastHit> Raycast::castLidarScan(
    const scene::SimScene& scene,
    const Vec3& origin,
    const Vec3& forwardDir,
    uint32_t numRays,
    float fovDegrees,
    float maxDistance
) noexcept {
    std::vector<RaycastHit> scanResults;
    scanResults.reserve(numRays);

    // Pre-build world AABB list once — avoids recomputing scaledMin/Max
    // N_entities times per ray (previously O(numRays × N_entities) multiplications).
    const auto& entities = scene.entities();
    struct CachedAABB { Vec3 min, max; bool valid; };
    std::vector<CachedAABB> aabbs;
    aabbs.reserve(entities.size());
    for (const auto& entity : entities) {
        Vec3 sMin{entity.localBounds.min.x * entity.scale.x,
                  entity.localBounds.min.y * entity.scale.y,
                  entity.localBounds.min.z * entity.scale.z};
        Vec3 sMax{entity.localBounds.max.x * entity.scale.x,
                  entity.localBounds.max.y * entity.scale.y,
                  entity.localBounds.max.z * entity.scale.z};
        aabbs.push_back({entity.position + sMin, entity.position + sMax, true});
    }

    float startAngle = -(fovDegrees * 0.5f) * DEG2RAD;
    float angleStep  = (fovDegrees * DEG2RAD) / static_cast<float>(numRays > 1 ? numRays : 1);
    float baseYaw    = std::atan2(forwardDir.x, forwardDir.z);

    for (uint32_t i = 0; i < numRays; ++i) {
        float currentAngle = baseYaw + startAngle + (angleStep * static_cast<float>(i));
        Vec3 rayDir{std::sin(currentAngle), 0.0f, std::cos(currentAngle)};
        Vec3 dirNorm = rayDir.normalized();

        // Inline ray-AABB test against pre-built cache (no per-ray entity loop)
        RaycastHit bestHit{};
        bestHit.distance = maxDistance;

        for (std::size_t j = 0; j < aabbs.size(); ++j) {
            float t = 0.0f;
            if (intersectAABB(origin, dirNorm, aabbs[j].min, aabbs[j].max, maxDistance, t)) {
                if (t > 0.001f && t < bestHit.distance) {
                    bestHit.hit      = true;
                    bestHit.distance = t;
                    bestHit.point    = origin + (dirNorm * t);
                    bestHit.entityName = entities[j].name;
                    Vec3 center = (aabbs[j].min + aabbs[j].max) * 0.5f;
                    bestHit.normal = (bestHit.point - center).normalized();
                }
            }
        }

        scanResults.push_back(bestHit);
    }

    return scanResults;
}

} // namespace corium_sim::physics
