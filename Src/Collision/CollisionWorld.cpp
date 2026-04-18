#include "CollisionWorld.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <print>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"
#include "Solver/PenetrationVectorSolver.hpp"

namespace
{
    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionCircleVsCircle(const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                                     const std::shared_ptr<Guch2D::CollisionBody>& bodyB)
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

        collisionPoints.ContactPoints.front() = centerA + directionAB * radiusA;
        collisionPoints.ContactPoints.back() = centerB - directionAB * radiusB;
        collisionPoints.Normal = -directionAB;
        collisionPoints.Depth = std::max(0.0F, radiusSum - distance);

        return collisionPoints;
    }

    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionAABBVsAABB(const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                                 const std::shared_ptr<Guch2D::CollisionBody>& bodyB)
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

            collisionPoints.ContactPoints.front() = {centerA.x + (extentA.x * direction),
                                                     minOverlapY};
            collisionPoints.ContactPoints.back() = {centerB.x - (extentB.x * direction),
                                                    maxOverlapY};
        }
        else
        {
            const float direction = delta.y >= 0.0F ? 1.0F : -1.0F;
            collisionPoints.Normal = {0.0F, -direction};
            collisionPoints.Depth = overlapY;

            const float minOverlapX = std::max(centerA.x - extentA.x, centerB.x - extentB.x);
            const float maxOverlapX = std::min(centerA.x + extentA.x, centerB.x + extentB.x);

            collisionPoints.ContactPoints.front() = {minOverlapX,
                                                     centerA.y + (extentA.y * direction)};
            collisionPoints.ContactPoints.back() = {maxOverlapX,
                                                    centerB.y - (extentB.y * direction)};
        }

        return collisionPoints;
    }

    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionAABBVsCircle(const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                                   const std::shared_ptr<Guch2D::CollisionBody>& bodyB)
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
            collisionPoints.ContactPoints.front() = closestPoint;
            collisionPoints.ContactPoints.back() = centerB + (collisionPoints.Normal * radiusB);
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
        collisionPoints.ContactPoints.front() = {minX, centerB.y};

        if (distanceToRight < minDistanceToFace)
        {
            minDistanceToFace = distanceToRight;
            collisionPoints.Normal = {-1.0F, 0.0F};
            collisionPoints.ContactPoints.front() = {maxX, centerB.y};
        }

        if (distanceToBottom < minDistanceToFace)
        {
            minDistanceToFace = distanceToBottom;
            collisionPoints.Normal = {0.0F, 1.0F};
            collisionPoints.ContactPoints.front() = {centerB.x, minY};
        }

        if (distanceToTop < minDistanceToFace)
        {
            minDistanceToFace = distanceToTop;
            collisionPoints.Normal = {0.0F, -1.0F};
            collisionPoints.ContactPoints.front() = {centerB.x, maxY};
        }

        collisionPoints.Depth = radiusB + minDistanceToFace;
        collisionPoints.ContactPoints.back() = centerB + (collisionPoints.Normal * radiusB);

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
}   // namespace

namespace Guch2D
{
    void CollisionWorld::Step() const
    {
        FindCollisions();
        InvokeBeginOverlap();
        InvokeEndOverlap();

        SolveCollisions();

        _previousCollisions = std::move(_collisions);
    }

    void CollisionWorld::FindCollisions() const
    {
        _collisions.clear();

        for (const auto& objectA : _objects)
        {
            for (const auto& objectB : _objects)
            {
                if (objectA == objectB)
                    break;

                const auto collisionPoints = CheckCollisions(objectA, objectB);
                if (!collisionPoints.HasCollision)
                    continue;

                const auto collision = Collision(objectA, objectB, collisionPoints);
                _collisions.emplace_back(collision);
            }
        }
    }

    void CollisionWorld::SolveCollisions() const
    {
        for (const auto& solver : _solvers)
        {
            if (solver)
                solver->Solve(_collisions);
        }
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

    CollisionPoints CollisionWorld::CheckCollisions(std::shared_ptr<CollisionBody> bodyA,
                                                    std::shared_ptr<CollisionBody> bodyB)
    {
        constexpr auto typeCount = static_cast<size_t>(ColliderType::Count);

        using CollisionFunc = std::function<CollisionPoints(const std::shared_ptr<CollisionBody>&,
                                                            const std::shared_ptr<CollisionBody>&)>;
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
            std::println("[Warning] CollisionWorld::CheckCollisions: Invalid collider type");
        }

        return {};
    }
}   // namespace Guch2D
