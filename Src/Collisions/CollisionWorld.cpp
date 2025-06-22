#include "CollisionWorld.hpp"

#include <array>
#include <functional>

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

        // Get difference vector between both centers
        Vect difference = circleCenter - aabbCenter;

        // Clamp difference to AABB extents
        Vect clamped = {std::max(-halfExtents.x, std::min(difference.x, halfExtents.x)),
                        std::max(-halfExtents.y, std::min(difference.y, halfExtents.y))};

        // Closest point on/in the AABB to the circle center
        Vect closest = aabbCenter + clamped;

        // Vector from circle center to closest point
        Vect circleToBox = closest - circleCenter;

        if (circleToBox.LengthSquared() < radius)
        {
            assert(false && "Collision detected between CircleCollider and AABBCollider!");
        }
    }

}   // namespace

namespace Guch2D
{
    auto CollisionWorld::Step() noexcept -> void
    {
        ResolveCollisions();
    }

    auto CollisionWorld::ResolveCollisions() noexcept -> void
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
        static const std::array<std::function<void(const std::shared_ptr<CollisionBody>&,
                                                   const std::shared_ptr<CollisionBody>&)>,
                                2>
            collisionCheckFunction = {CheckCollisionCircleCircle, CheckCollisionCircleAABBCollider};

        const auto collisionFunctionIndex = objectA->GetCollider()->GetType()
                                          * objectB->GetCollider()->GetType();

        // No collision function for None type
        if (collisionFunctionIndex == 0) return;

        (collisionCheckFunction.at(collisionFunctionIndex - 1))(objectA, objectB);
    }
}   // namespace Guch2D
