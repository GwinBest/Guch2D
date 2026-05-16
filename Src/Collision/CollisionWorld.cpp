#include "CollisionWorld.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <print>
#include <ranges>
#include <source_location>
#include <unordered_map>
#include <utility>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/StaticRigidBody.hpp"
#include "Math/Vector.hpp"
#include "Solver/PositionSolver.hpp"
#include "Utils/Logger.hpp"

namespace
{
    using SpatialBucket = std::vector<Guch2D::CollisionWorld::ObjectType>;
    using SpatialGridMap = std::unordered_map<size_t, SpatialBucket>;
    constexpr size_t SpatialHashingCalculateHash(int64_t cellX, int64_t cellY) noexcept;

    [[nodiscard]] std::shared_ptr<Guch2D::DynamicRigidBody>
        GetDynamicRigidBody(const Guch2D::CollisionWorld::ObjectType& object)
    {
        return std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(object);
    }

    [[nodiscard]] bool IsSleepingDynamicRigidBody(const Guch2D::CollisionWorld::ObjectType& object)
    {
        const auto dynamicRigidBody = GetDynamicRigidBody(object);
        return dynamicRigidBody && !dynamicRigidBody->IsAwake();
    }

    [[nodiscard]] bool CanWakeSleepingRigidBody(const Guch2D::CollisionWorld::ObjectType& object)
    {
        if (!object)
            return false;

        if (const auto dynamicRigidBody = GetDynamicRigidBody(object))
            return dynamicRigidBody->IsAwake();

        return !std::dynamic_pointer_cast<Guch2D::StaticRigidBody>(object);
    }

    [[nodiscard]] Guch2D::Vect GetWakeVelocity(const Guch2D::CollisionWorld::ObjectType& object)
    {
        if (const auto dynamicRigidBody = GetDynamicRigidBody(object);
            dynamicRigidBody && dynamicRigidBody->IsAwake())
        {
            return dynamicRigidBody->GetVelocity();
        }

        return {0.0F, 0.0F};
    }

    [[nodiscard]] bool HasWakeImpact(const Guch2D::CollisionWorld::ObjectType& objectA,
                                     const Guch2D::CollisionWorld::ObjectType& objectB,
                                     const Guch2D::CollisionPoints& collisionPoints)
    {
        constexpr float wakePenetrationDepthThreshold = 0.05F;
        constexpr float wakeApproachSpeedThreshold = 0.2F;

        if (collisionPoints.Depth >= wakePenetrationDepthThreshold)
            return true;

        const Guch2D::Vect velocityA = GetWakeVelocity(objectA);
        const Guch2D::Vect velocityB = GetWakeVelocity(objectB);
        const float approachSpeed = Guch2D::VectDot(velocityB - velocityA, collisionPoints.Normal);
        return approachSpeed >= wakeApproachSpeedThreshold;
    }

    [[nodiscard]] bool ShouldProcessCollisionPair(const Guch2D::CollisionWorld::ObjectType& objectA,
                                                  const Guch2D::CollisionWorld::ObjectType& objectB)
    {
        if (!objectA || !objectB || objectA == objectB)
            return false;

        const bool objectAIsSleeping = IsSleepingDynamicRigidBody(objectA);
        const bool objectBIsSleeping = IsSleepingDynamicRigidBody(objectB);
        if (!objectAIsSleeping && !objectBIsSleeping)
            return true;

        if (objectAIsSleeping && objectBIsSleeping)
            return false;

        return objectAIsSleeping ? CanWakeSleepingRigidBody(objectB)
                                 : CanWakeSleepingRigidBody(objectA);
    }

    void WakeSleepingRigidBodies(const Guch2D::CollisionWorld::ObjectType& objectA,
                                 const Guch2D::CollisionWorld::ObjectType& objectB,
                                 const Guch2D::CollisionPoints& collisionPoints)
    {
        const auto dynamicRigidBodyA = GetDynamicRigidBody(objectA);
        const auto dynamicRigidBodyB = GetDynamicRigidBody(objectB);

        const bool hasWakeImpact = HasWakeImpact(objectA, objectB, collisionPoints);
        if (dynamicRigidBodyA && !dynamicRigidBodyA->IsAwake() && CanWakeSleepingRigidBody(objectB)
            && hasWakeImpact)
        {
            dynamicRigidBodyA->SetAwake(true);
        }

        if (dynamicRigidBodyB && !dynamicRigidBodyB->IsAwake() && CanWakeSleepingRigidBody(objectA)
            && hasWakeImpact)
        {
            dynamicRigidBodyB->SetAwake(true);
        }
    }

    [[nodiscard]] std::pair<int64_t, int64_t>
        GetSpatialCellFromPosition(const Guch2D::Vect& position,
                                   const float cellSizeX,
                                   const float cellSizeY)
    {
        return {
            static_cast<int64_t>(std::floor(position.x / cellSizeX)),
            static_cast<int64_t>(std::floor(position.y / cellSizeY)),
        };
    }

    [[nodiscard]] std::pair<int64_t, int64_t>
        GetSpatialCell(const Guch2D::CollisionWorld::ObjectType& object,
                       const float cellSizeX,
                       const float cellSizeY)
    {
        return GetSpatialCellFromPosition(object->GetPosition(), cellSizeX, cellSizeY);
    }

    void AppendSameBucketCollisions(const SpatialBucket& bucket,
                                    std::vector<Guch2D::Collision>& possibleCollisions)
    {
        for (size_t i = 0; i < bucket.size(); ++i)
        {
            for (size_t j = i + 1; j < bucket.size(); ++j)
            {
                if (!ShouldProcessCollisionPair(bucket.at(i), bucket.at(j)))
                    continue;

                possibleCollisions.emplace_back(bucket.at(i), bucket.at(j));
            }
        }
    }

    void AppendCrossBucketCollisions(const SpatialBucket& bucketA,
                                     const SpatialBucket& bucketB,
                                     std::vector<Guch2D::Collision>& possibleCollisions)
    {
        for (const auto& objectA : bucketA)
        {
            for (const auto& objectB : bucketB)
            {
                if (!ShouldProcessCollisionPair(objectA, objectB))
                    continue;

                possibleCollisions.emplace_back(objectA, objectB);
            }
        }
    }

    void AppendNeighborBucketCollisions(const SpatialGridMap& gridMap,
                                        const SpatialBucket& bucket,
                                        const int64_t cellX,
                                        const int64_t cellY,
                                        std::vector<Guch2D::Collision>& possibleCollisions)
    {
        static constexpr std::array<std::pair<int64_t, int64_t>, 9> NeighborOffsets = {
            {
             {-1, -1},
             {-1, 0},
             {-1, 1},
             {0, -1},
             {0, 0},
             {0, 1},
             {1, -1},
             {1, 0},
             {1, 1},
             }
        };

        for (const auto& [dxOffset, dyOffset] : NeighborOffsets)
        {
            const size_t neighborHash = SpatialHashingCalculateHash(cellX + dxOffset,
                                                                    cellY + dyOffset);
            auto iterator = gridMap.find(neighborHash);

            if (iterator == gridMap.end())
                continue;

            AppendCrossBucketCollisions(bucket, iterator->second, possibleCollisions);
        }
    }

    [[nodiscard]] std::array<const Guch2D::CollisionBody*, 2>
        GetCanonicalCollisionPair(const Guch2D::Collision& collision)
    {
        const auto* bodyA = collision.BodyA.lock().get();
        const auto* bodyB = collision.BodyB.lock().get();

        if (bodyB < bodyA)
            std::swap(bodyA, bodyB);

        return {bodyA, bodyB};
    }

    struct RaycastIntersection final
    {
        Guch2D::Vect Point = {0.0F, 0.0F};
        Guch2D::Vect Normal = {0.0F, 0.0F};
        float Distance = 0.0F;
        bool HasHit = false;
    };

    [[nodiscard]] bool IsValidRaycastMaxDistance(const float maxDistance) noexcept
    {
        if (maxDistance < 0.0F)
            return false;

        if (Guch2D::IsFinite(maxDistance))
            return true;

        return maxDistance == std::numeric_limits<float>::infinity();
    }

    [[nodiscard]] bool IsValidRaycastInput(const Guch2D::Vect& origin,
                                           const Guch2D::Vect& direction,
                                           const float maxDistance) noexcept
    {
        if (!Guch2D::IsFinite(origin) || !Guch2D::IsFinite(direction))
            return false;

        if (!IsValidRaycastMaxDistance(maxDistance))
            return false;

        return Guch2D::VectLength(direction) > 0.0F;
    }

    [[nodiscard]] bool IsRaycastDistanceInRange(const float distance,
                                                const float maxDistance) noexcept
    {
        if (distance < 0.0F || !Guch2D::IsFinite(distance))
            return false;

        if (!Guch2D::IsFinite(maxDistance))
            return true;

        const float scaledEpsilon = std::numeric_limits<float>::epsilon()
                                  * std::max({1.0F, distance, maxDistance});
        return distance <= (maxDistance + scaledEpsilon);
    }

    [[nodiscard]] RaycastIntersection RaycastCircle(const Guch2D::CollisionWorld::ObjectType& body,
                                                    const Guch2D::Vect& origin,
                                                    const Guch2D::Vect& directionNormalized,
                                                    const float maxDistance)
    {
        const auto circleCollider = std::dynamic_pointer_cast<Guch2D::CircleCollider>(
            body ? body->GetCollider() : nullptr);
        if (!circleCollider)
            return {};

        const Guch2D::Vect center = body->GetColliderCenterWorld();
        const float radius = circleCollider->GetRadius();

        const Guch2D::Vect originToCenter = origin - center;
        const float bHalf = Guch2D::VectDot(originToCenter, directionNormalized);
        const float centerDistanceSquaredMinusRadiusSquared = Guch2D::VectDot(originToCenter,
                                                                              originToCenter)
                                                            - (radius * radius);

        const float discriminant = (bHalf * bHalf) - centerDistanceSquaredMinusRadiusSquared;
        const float scaledEpsilon = std::numeric_limits<float>::epsilon()
                                  * std::max({1.0F,
                                              std::abs(bHalf),
                                              std::abs(centerDistanceSquaredMinusRadiusSquared)});
        if (discriminant < -scaledEpsilon)
            return {};

        const float sqrtDiscriminant = std::sqrt(std::max(0.0F, discriminant));
        const float nearDistance = -bHalf - sqrtDiscriminant;
        const float farDistance = -bHalf + sqrtDiscriminant;

        const float hitDistance = nearDistance >= 0.0F ? nearDistance : farDistance;
        if (!IsRaycastDistanceInRange(hitDistance, maxDistance))
            return {};

        RaycastIntersection hit;
        hit.Distance = hitDistance;
        hit.Point = origin + directionNormalized * hitDistance;
        hit.Normal = Guch2D::VectNormalize(hit.Point - center);
        hit.HasHit = true;
        return hit;
    }

    [[nodiscard]] RaycastIntersection RaycastAABB(const Guch2D::CollisionWorld::ObjectType& body,
                                                  const Guch2D::Vect& origin,
                                                  const Guch2D::Vect& directionNormalized,
                                                  const float maxDistance)
    {
        const auto aabbCollider = std::dynamic_pointer_cast<Guch2D::AABBCollider>(
            body ? body->GetCollider() : nullptr);
        if (!aabbCollider)
            return {};

        const Guch2D::Vect center = body->GetColliderCenterWorld();
        const Guch2D::Vect extent = aabbCollider->GetExtent();

        const float minX = std::min(center.x - extent.x, center.x + extent.x);
        const float maxX = std::max(center.x - extent.x, center.x + extent.x);
        const float minY = std::min(center.y - extent.y, center.y + extent.y);
        const float maxY = std::max(center.y - extent.y, center.y + extent.y);

        float distanceToEnter = -std::numeric_limits<float>::infinity();
        float distanceToExit = std::numeric_limits<float>::infinity();
        Guch2D::Vect enterNormal = {0.0F, 0.0F};
        Guch2D::Vect exitNormal = {0.0F, 0.0F};
        constexpr float parallelEpsilon = 1.0e-6F;

        struct SlabInput
        {
            float OriginAxis = 0.0F;
            float DirectionAxis = 0.0F;
            float MinimumAxis = 0.0F;
            float MaximumAxis = 0.0F;
            Guch2D::Vect MinimumNormal;
            Guch2D::Vect MaximumNormal;
        };

        const auto intersectSlab = [&](const SlabInput& slab) -> bool {
            if (std::abs(slab.DirectionAxis) <= parallelEpsilon)
                return slab.OriginAxis >= slab.MinimumAxis && slab.OriginAxis <= slab.MaximumAxis;

            const float inverseDirectionAxis = 1.0F / slab.DirectionAxis;
            float nearDistance = (slab.MinimumAxis - slab.OriginAxis) * inverseDirectionAxis;
            float farDistance = (slab.MaximumAxis - slab.OriginAxis) * inverseDirectionAxis;

            Guch2D::Vect nearNormal = slab.MinimumNormal;
            Guch2D::Vect farNormal = slab.MaximumNormal;
            if (nearDistance > farDistance)
            {
                std::swap(nearDistance, farDistance);
                std::swap(nearNormal, farNormal);
            }

            if (nearDistance > distanceToEnter)
            {
                distanceToEnter = nearDistance;
                enterNormal = nearNormal;
            }

            if (farDistance < distanceToExit)
            {
                distanceToExit = farDistance;
                exitNormal = farNormal;
            }

            return distanceToEnter <= distanceToExit;
        };

        if (!intersectSlab({
                .OriginAxis = origin.x,
                .DirectionAxis = directionNormalized.x,
                .MinimumAxis = minX,
                .MaximumAxis = maxX,
                .MinimumNormal = {-1.0F, 0.0F},
                .MaximumNormal = {1.0F, 0.0F}
        }))
        {
            return {};
        }

        if (!intersectSlab({
                .OriginAxis = origin.y,
                .DirectionAxis = directionNormalized.y,
                .MinimumAxis = minY,
                .MaximumAxis = maxY,
                .MinimumNormal = {0.0F, -1.0F},
                .MaximumNormal = {0.0F, 1.0F}
        }))
        {
            return {};
        }

        if (distanceToExit < 0.0F)
            return {};

        const bool startsInside = distanceToEnter < 0.0F;
        const float hitDistance = startsInside ? distanceToExit : distanceToEnter;
        if (!IsRaycastDistanceInRange(hitDistance, maxDistance))
            return {};

        RaycastIntersection hit;
        hit.Distance = hitDistance;
        hit.Point = origin + directionNormalized * hitDistance;
        hit.Normal = startsInside ? exitNormal : enterNormal;
        hit.HasHit = true;
        return hit;
    }

    [[nodiscard]] RaycastIntersection RaycastObject(const Guch2D::CollisionWorld::ObjectType& body,
                                                    const Guch2D::Vect& origin,
                                                    const Guch2D::Vect& directionNormalized,
                                                    const float maxDistance)
    {
        if (!body)
            return {};

        const auto& collider = body->GetCollider();
        if (!collider)
            return {};

        switch (collider->GetColliderType())
        {
        case Guch2D::ColliderType::Circle:
        {
            return RaycastCircle(body, origin, directionNormalized, maxDistance);
        }
        case Guch2D::ColliderType::AABB:
        {
            return RaycastAABB(body, origin, directionNormalized, maxDistance);
        }
        default:
        {
            break;
        }
        }

        return {};
    }

    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionCircleVsCircle(const Guch2D::CollisionWorld::ObjectType& bodyA,
                                     const Guch2D::CollisionWorld::ObjectType& bodyB)
    {
        const auto centerA = bodyA->GetColliderCenterWorld();
        const auto centerB = bodyB->GetColliderCenterWorld();
        const auto radiusA = std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())
                                 ->GetRadius();
        const auto radiusB = std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())
                                 ->GetRadius();

        const auto delta = centerB - centerA;
        const float radiusSum = radiusA + radiusB;
        const float distanceSquared = (delta.x * delta.x) + (delta.y * delta.y);
        const float radiusSumSquared = radiusSum * radiusSum;
        const float scaledEpsilon = std::numeric_limits<float>::epsilon()
                                  * std::max({1.0F, distanceSquared, radiusSumSquared});

        Guch2D::CollisionPoints collisionPoints;
        collisionPoints.HasCollision = distanceSquared <= (radiusSumSquared + scaledEpsilon);

        if (!collisionPoints.HasCollision)
            return collisionPoints;

        const float distance = std::sqrt(distanceSquared);
        const Guch2D::Vect directionAB = distance > 0.0F ? delta / distance
                                                         : Guch2D::Vect {1.0F, 0.0F};

        collisionPoints.ContactPoints.front().Position = centerA + directionAB * radiusA;
        collisionPoints.ContactPoints.back().Position = centerB - directionAB * radiusB;
        collisionPoints.Normal = -directionAB;
        collisionPoints.Depth = std::max(0.0F, radiusSum - distance);

        return collisionPoints;
    }

    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionAABBVsAABB(const Guch2D::CollisionWorld::ObjectType& bodyA,
                                 const Guch2D::CollisionWorld::ObjectType& bodyB)
    {
        const auto centerA = bodyA->GetColliderCenterWorld();
        const auto centerB = bodyB->GetColliderCenterWorld();
        const auto extentA = std::dynamic_pointer_cast<Guch2D::AABBCollider>(bodyA->GetCollider())
                                 ->GetExtent();
        const auto extentB = std::dynamic_pointer_cast<Guch2D::AABBCollider>(bodyB->GetCollider())
                                 ->GetExtent();
        const float deltaX = std::abs(centerA.x - centerB.x);
        const float deltaY = std::abs(centerA.y - centerB.y);
        const float sumExtentX = extentA.x + extentB.x;
        const float sumExtentY = extentA.y + extentB.y;
        const float scaledEpsilonX = std::numeric_limits<float>::epsilon()
                                   * std::max({1.0F, deltaX, sumExtentX});
        const float scaledEpsilonY = std::numeric_limits<float>::epsilon()
                                   * std::max({1.0F, deltaY, sumExtentY});

        Guch2D::CollisionPoints collisionPoints;
        collisionPoints.HasCollision = deltaX <= (sumExtentX + scaledEpsilonX)
                                    && deltaY <= (sumExtentY + scaledEpsilonY);

        if (!collisionPoints.HasCollision)
            return collisionPoints;

        const auto delta = centerB - centerA;

        const float overlapX = std::max(0.0F, sumExtentX - deltaX);
        const float overlapY = std::max(0.0F, sumExtentY - deltaY);

        // Resolve collision along the axis with the smaller penetration depth.
        if (overlapX < overlapY)
        {
            const float direction = delta.x >= 0.0F ? 1.0F : -1.0F;
            collisionPoints.Normal = {-direction, 0.0F};
            collisionPoints.Depth = overlapX;

            const float minOverlapY = std::max(centerA.y - extentA.y, centerB.y - extentB.y);
            const float maxOverlapY = std::min(centerA.y + extentA.y, centerB.y + extentB.y);

            collisionPoints.ContactPoints.front().Position = {centerA.x + (extentA.x * direction),
                                                              minOverlapY};
            collisionPoints.ContactPoints.back().Position = {centerB.x - (extentB.x * direction),
                                                             maxOverlapY};
        }
        else
        {
            const float direction = delta.y >= 0.0F ? 1.0F : -1.0F;
            collisionPoints.Normal = {0.0F, -direction};
            collisionPoints.Depth = overlapY;

            const float minOverlapX = std::max(centerA.x - extentA.x, centerB.x - extentB.x);
            const float maxOverlapX = std::min(centerA.x + extentA.x, centerB.x + extentB.x);

            collisionPoints.ContactPoints.front().Position = {minOverlapX,
                                                              centerA.y + (extentA.y * direction)};
            collisionPoints.ContactPoints.back().Position = {maxOverlapX,
                                                             centerB.y - (extentB.y * direction)};
        }

        return collisionPoints;
    }

    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionAABBVsCircle(const Guch2D::CollisionWorld::ObjectType& bodyA,
                                   const Guch2D::CollisionWorld::ObjectType& bodyB)
    {
        const auto centerA = bodyA->GetColliderCenterWorld();
        const auto centerB = bodyB->GetColliderCenterWorld();
        const auto extentA = std::dynamic_pointer_cast<Guch2D::AABBCollider>(bodyA->GetCollider())
                                 ->GetExtent();
        const auto radiusB = std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())
                                 ->GetRadius();

        Guch2D::CollisionPoints collisionPoints;

        const float minX = centerA.x - extentA.x;
        const float maxX = centerA.x + extentA.x;
        const float minY = centerA.y - extentA.y;
        const float maxY = centerA.y + extentA.y;

        const auto closestX = std::clamp(centerB.x, minX, maxX);
        const auto closestY = std::clamp(centerB.y, minY, maxY);
        const Guch2D::Vect closestPoint = {closestX, closestY};
        const auto delta = closestPoint - centerB;
        const float distanceSquared = (delta.x * delta.x) + (delta.y * delta.y);
        const float radiusSquared = radiusB * radiusB;
        const float scaledEpsilon = std::numeric_limits<float>::epsilon()
                                  * std::max({1.0F, distanceSquared, radiusSquared});

        collisionPoints.HasCollision = distanceSquared <= (radiusSquared + scaledEpsilon);

        if (!collisionPoints.HasCollision)
            return collisionPoints;

        if (distanceSquared > 0.0F)
        {
            const float distance = std::sqrt(distanceSquared);
            collisionPoints.Normal = delta / distance;
            collisionPoints.Depth = std::max(0.0F, radiusB - distance);
            collisionPoints.ContactPoints.front().Position = closestPoint;
            collisionPoints.ContactPoints.back().Position = centerB
                                                          + (collisionPoints.Normal * radiusB);
            return collisionPoints;
        }

        // Circle center is inside the AABB (or exactly on the boundary).
        // Choose the nearest face and orient normal to match solver expectations.
        const float distanceToLeft = centerB.x - minX;
        const float distanceToRight = maxX - centerB.x;
        const float distanceToBottom = centerB.y - minY;
        const float distanceToTop = maxY - centerB.y;

        float minDistanceToFace = distanceToLeft;
        collisionPoints.Normal = {1.0F, 0.0F};
        collisionPoints.ContactPoints.front().Position = {minX, centerB.y};

        if (distanceToRight < minDistanceToFace)
        {
            minDistanceToFace = distanceToRight;
            collisionPoints.Normal = {-1.0F, 0.0F};
            collisionPoints.ContactPoints.front().Position = {maxX, centerB.y};
        }

        if (distanceToBottom < minDistanceToFace)
        {
            minDistanceToFace = distanceToBottom;
            collisionPoints.Normal = {0.0F, 1.0F};
            collisionPoints.ContactPoints.front().Position = {centerB.x, minY};
        }

        if (distanceToTop < minDistanceToFace)
        {
            minDistanceToFace = distanceToTop;
            collisionPoints.Normal = {0.0F, -1.0F};
            collisionPoints.ContactPoints.front().Position = {centerB.x, maxY};
        }

        collisionPoints.Depth = radiusB + minDistanceToFace;
        collisionPoints.ContactPoints.back().Position = centerB
                                                      + (collisionPoints.Normal * radiusB);

        return collisionPoints;
    }

    [[nodiscard]] bool HasSameOverlapPair(const Guch2D::Collision& lhs,
                                          const Guch2D::Collision& rhs)
    {
        const auto lhsA = lhs.BodyA.lock();
        const auto lhsB = lhs.BodyB.lock();
        const auto rhsA = rhs.BodyA.lock();
        const auto rhsB = rhs.BodyB.lock();

        if (!lhsA || !lhsB || !rhsA || !rhsB)
            return false;

        return (lhsA == rhsA && lhsB == rhsB) || (lhsA == rhsB && lhsB == rhsA);
    }

    constexpr size_t SpatialHashingCalculateHash(const int64_t cellX, const int64_t cellY) noexcept
    {
        constexpr auto hashValueX = 73856093;
        constexpr auto hashValueY = 19349663;
        return static_cast<size_t>(cellX * hashValueX) ^ static_cast<size_t>(cellY * hashValueY);
    }
}   // namespace

namespace Guch2D
{
    void CollisionWorld::Step()
    {
        _collisions.clear();

        const auto possibleCollisions = BroadPhase();
        NarrowPhase(possibleCollisions);

        InvokeBeginOverlap();
        InvokeEndOverlap();

        SolveCollisions();

        _previousCollisions = std::move(_collisions);
    }

    std::vector<Collision> CollisionWorld::BroadPhase()
    {
        switch (_broadPhaseType)
        {
        case BroadPhaseType::SweepAndPrune:
        {
            return SweepAndPrune();
        }
        case BroadPhaseType::SpatialHashing:
        {
            return SpatialHashing();
        }
        default:
        {
            WarnLog("Unknown broad phase type");
            break;
        }
        }

        return {};
    }

    void CollisionWorld::NarrowPhase(const std::vector<Collision>& possibleCollisions) const
    {
        for (const auto& collision : possibleCollisions)
        {
            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();
            if (!ShouldProcessCollisionPair(bodyA, bodyB))
                continue;

            const auto collisionPoints = CheckCollisions(bodyA, bodyB);

            if (!collisionPoints.HasCollision)
                continue;

            WakeSleepingRigidBodies(bodyA, bodyB, collisionPoints);
            _collisions.emplace_back(bodyA, bodyB, collisionPoints);
        }

        std::erase_if(_collisions,
                      [](const Collision& collision) { return !collision.Points.HasCollision; });
    }

    void CollisionWorld::InvokeBeginOverlap() const
    {
        // Invoke OnBeginOverlap if this collision was not present in the previous frame
        std::ranges::for_each(_collisions, [&](const Collision& collision) {
            if (std::ranges::any_of(_previousCollisions, [&](const Collision& previousCollision) {
                    return HasSameOverlapPair(collision, previousCollision);
                }))
            {
                return;
            }

            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();

            if (bodyA)
                bodyA->InvokeOnBeginOverlap(collision);
            if (bodyB)
                bodyB->InvokeOnBeginOverlap(collision);
        });
    }

    void CollisionWorld::InvokeEndOverlap() const
    {
        // Invoke OnEndOverlap if this collision was present in the previous frame
        std::ranges::for_each(_previousCollisions, [&](const Collision& collision) {
            if (std::ranges::any_of(_collisions, [&](const Collision& currentCollision) {
                    return HasSameOverlapPair(collision, currentCollision);
                }))
            {
                return;
            }

            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();

            if (bodyA)
                bodyA->InvokeOnEndOverlap(collision);
            if (bodyB)
                bodyB->InvokeOnEndOverlap(collision);
        });
    }

    void CollisionWorld::SolveCollisions() const
    {
        for (const auto& solver : _solvers)
        {
            if (solver)
                solver->Solve(_collisions);
        }
    }

    CollisionPoints CollisionWorld::CheckCollisions(ObjectType bodyA, ObjectType bodyB)
    {
        constexpr auto typeCount = static_cast<uint8_t>(ColliderType::Count);

        using CollisionFunc = std::function<CollisionPoints(const ObjectType&, const ObjectType&)>;
        using CollisionFuncMatrix = std::array<std::array<CollisionFunc, typeCount>, typeCount>;

        static const CollisionFuncMatrix CollisionCheckMatrix = {
            {
             //     None          Circle      AABB
                {nullptr, nullptr, nullptr},                                             // None
                {nullptr, &CheckCollisionCircleVsCircle, &CheckCollisionAABBVsCircle},   // Circle
                {nullptr, &CheckCollisionAABBVsCircle, &CheckCollisionAABBVsAABB},       // AABB
            }
        };

        if (!bodyA || !bodyB)
            return {};

        const auto& bodyACollider = bodyA->GetCollider();
        const auto& bodyBCollider = bodyB->GetCollider();

        if (!bodyACollider || !bodyBCollider)
            return {};

        const auto typeA = static_cast<uint8_t>(bodyACollider->GetColliderType());
        const auto typeB = static_cast<uint8_t>(bodyBCollider->GetColliderType());

        // Ensure a consistent body order for collision functions: if B's type is greater,
        // swap A/B. Collision points are swapped back below to match the original order.
        const bool swap = typeB > typeA;
        if (swap)
        {
            std::swap(bodyA, bodyB);
        }

        try
        {
            if (const auto& collisionFunc = CollisionCheckMatrix.at(typeA).at(typeB); collisionFunc)
            {
                auto collisionPoints = collisionFunc(bodyA, bodyB);

                // Swap back collision points
                if (swap)
                {
                    std::swap(collisionPoints.ContactPoints.front(),
                              collisionPoints.ContactPoints.back());
                    collisionPoints.Normal = -collisionPoints.Normal;
                }

                return collisionPoints;
            }
        }
        catch (std::out_of_range&)
        {
            WarnLog("Invalid collider type");
        }

        return {};
    }

    RaycastHit CollisionWorld::Raycast(const Vect& origin,
                                       const Vect& direction,
                                       const float maxDistance) const
    {
        if (!IsValidRaycastInput(origin, direction, maxDistance))
            return {};

        const Vect directionNormalized = VectNormalize(direction);
        float bestDistance = std::numeric_limits<float>::infinity();
        RaycastHit bestHit;

        for (const auto& object : _objects)
        {
            const auto hit = RaycastObject(object, origin, directionNormalized, maxDistance);
            if (!hit.HasHit)
                continue;

            if (hit.Distance < bestDistance)
            {
                bestDistance = hit.Distance;
                bestHit.Body = object;
                bestHit.Point = hit.Point;
                bestHit.Normal = hit.Normal;
                bestHit.Distance = hit.Distance;
                bestHit.HasHit = true;
            }
        }

        return bestHit;
    }

    std::vector<RaycastHit> CollisionWorld::RaycastAll(const Vect& origin,
                                                       const Vect& direction,
                                                       const float maxDistance) const
    {
        if (!IsValidRaycastInput(origin, direction, maxDistance))
            return {};

        const Vect directionNormalized = VectNormalize(direction);
        std::vector<RaycastHit> hits;
        hits.reserve(_objects.size());

        for (const auto& object : _objects)
        {
            const auto [Point, Normal, Distance, HasHit] = RaycastObject(object,
                                                                         origin,
                                                                         directionNormalized,
                                                                         maxDistance);
            if (!HasHit)
                continue;

            RaycastHit raycastHit;
            raycastHit.Body = object;
            raycastHit.Point = Point;
            raycastHit.Normal = Normal;
            raycastHit.Distance = Distance;
            raycastHit.HasHit = true;
            hits.push_back(raycastHit);
        }

        std::ranges::sort(hits, std::ranges::less {}, [](const RaycastHit& hit) {
            return hit.Distance;
        });

        return hits;
    }

    std::vector<Collision> CollisionWorld::SweepAndPrune()
    {
        if (_objects.empty())
            return {};

        // sort object to its left border
        std::ranges::sort(_objects, std::ranges::less {}, [](const auto& body) {
            return body ? body->GetColliderLeftBorderWorld().x
                        : -std::numeric_limits<float>::infinity();
        });

        std::vector<Collision> possibleCollisions;

        for (size_t i = 0; i < _objects.size(); ++i)
        {
            for (size_t j = i + 1; j < _objects.size(); ++j)
            {
                if (_objects.at(i)->GetColliderRightBorderWorld().x
                    < _objects.at(j)->GetColliderLeftBorderWorld().x)
                    break;

                if (!ShouldProcessCollisionPair(_objects.at(i), _objects.at(j)))
                    continue;

                possibleCollisions.emplace_back(_objects.at(i), _objects.at(j));
            }
        }

        return possibleCollisions;
    }

    std::vector<Collision> CollisionWorld::SpatialHashing() const
    {
        // Cell size in meters (m)
        static constexpr float CellSizeX = 4.0F;
        static constexpr float CellSizeY = 4.0F;

        SpatialGridMap gridMap;

        for (const auto& object : _objects)
        {
            const auto [cellX, cellY] = GetSpatialCell(object, CellSizeX, CellSizeY);
            const size_t hash = SpatialHashingCalculateHash(cellX, cellY);
            gridMap[hash].emplace_back(object);
        }

        std::vector<Collision> possibleCollisions;

        for (const auto& bucket : gridMap | std::views::values)
        {
            if (bucket.empty())
                continue;

            AppendSameBucketCollisions(bucket, possibleCollisions);

            const auto [cellX, cellY] = GetSpatialCell(bucket.front(), CellSizeX, CellSizeY);
            AppendNeighborBucketCollisions(gridMap, bucket, cellX, cellY, possibleCollisions);
        }

        // Sort vector to remove duplicated collisions
        std::ranges::sort(possibleCollisions, std::ranges::less {}, [](const Collision& collision) {
            return GetCanonicalCollisionPair(collision);
        });

        const auto duplicatesRange = std::ranges::unique(
            possibleCollisions,
            std::ranges::equal_to {},
            [](const Collision& collision) { return GetCanonicalCollisionPair(collision); });
        possibleCollisions.erase(duplicatesRange.begin(), duplicatesRange.end());

        return possibleCollisions;
    }
}   // namespace Guch2D
