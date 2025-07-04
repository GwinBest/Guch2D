#include "CollisionWorld.hpp"

#include <functional>
#include <vector>

#include "Collisions/AABBCollider.hpp"
#include "Collisions/CircleCollider.hpp"

namespace
{
    using Guch2D::AABBCollider;
    using Guch2D::CircleCollider;
    using Guch2D::CollisionBody;
    using Guch2D::Vect;

    auto CheckCollisionCircleCircle(const std::shared_ptr<CollisionBody>& objectA,
                                    const std::shared_ptr<CollisionBody>& objectB) noexcept -> void
    {
        const auto circleA = std::dynamic_pointer_cast<CircleCollider>(objectA->GetCollider());
        const auto circleB = std::dynamic_pointer_cast<CircleCollider>(objectB->GetCollider());

        // Calculate the distance between the centers of the circles
        const Vect centerA = objectA->GetColliderCenterWorld();
        const Vect centerB = objectB->GetColliderCenterWorld();
        const float radiusA = circleA->GetRadius();
        const float radiusB = circleB->GetRadius();

        const float distanceSquared = (centerA - centerB).LengthSquared();

        const float radiusSum = radiusA + radiusB;
        if (distanceSquared <= radiusSum * radiusSum)
        {
            assert(false && "Collision detected between two CircleColliders!");
        }
    }

    auto CheckCollisionCircleAABBCollider(const std::shared_ptr<CollisionBody>& objectA,
                                          const std::shared_ptr<CollisionBody>& objectB) noexcept
        -> void
    {
        auto circle = std::dynamic_pointer_cast<CircleCollider>(objectA->GetCollider());
        auto aabb = std::dynamic_pointer_cast<AABBCollider>(objectB->GetCollider());

        if (!circle || !aabb)
        {
            circle = std::dynamic_pointer_cast<CircleCollider>(objectB->GetCollider());
            aabb = std::dynamic_pointer_cast<AABBCollider>(objectA->GetCollider());
        }

        // Get circle center and radius
        const Vect circleCenter = objectA->GetColliderCenterWorld();
        const float radius = circle->GetRadius();

        // Get AABB center and half-extents
        const Vect aabbCenter = objectB->GetColliderCenterWorld();
        const Vect halfExtents = aabb->GetExtends();

        // Get box closest point to sphere center
        const float closestX = std::max(aabbCenter.x - halfExtents.x,
                                        std::min(circleCenter.x, aabbCenter.x + halfExtents.x));
        const float closestY = std::max(aabbCenter.y - halfExtents.y,
                                        std::min(circleCenter.y, aabbCenter.y + halfExtents.y));

        const float distanceSquared = (closestX - circleCenter.x) * (closestX - circleCenter.x)
                                    + (closestY - circleCenter.y) * (closestY - circleCenter.y);

        if (distanceSquared <= radius * radius)
        {
            assert(false && "Collision detected between CircleColliders and AABB!");
        }
    }

}   // namespace

namespace Guch2D
{
    auto CollisionWorld::Step() -> void
    {
        ResolveCollisions();
    }

    auto CollisionWorld::ResolveCollisions() -> void
    {
        for (auto& objectA : _objects)
        {
            for (auto& objectB : _objects)
            {
                if (objectA == objectB) continue;

                // Check if both objects have colliders
                if (!objectA->GetCollider() || !objectB->GetCollider()) continue;

                CheckCollision(objectA, objectB);
            }
        }
    }

    auto CollisionWorld::CheckCollision(const std::shared_ptr<CollisionBody>& objectA,
                                        const std::shared_ptr<CollisionBody>& objectB) -> void
    {
        using CollisionFunc = std::function<void(const std::shared_ptr<CollisionBody>&,
                                                 const std::shared_ptr<CollisionBody>&)>;
        static const std::vector<std::vector<CollisionFunc>> collisionCheckMatrix = {
            // None,         Circle,                      AABB
            {nullptr, nullptr,                          nullptr                         }, // None
            {nullptr, CheckCollisionCircleCircle,       CheckCollisionCircleAABBCollider}, // Circle
            {nullptr, CheckCollisionCircleAABBCollider, nullptr                         }  // AABB
        };

        const auto typeA = objectA->GetCollider()->GetType();
        const auto typeB = objectB->GetCollider()->GetType();

        // No collision function for None type or unsupported pairs
        if (typeA == 0 || typeB == 0) return;

        const auto func = (collisionCheckMatrix.at(typeA)).at(typeB);
        if (func) func(objectA, objectB);
    }
}   // namespace Guch2D
