#include "CollisionWorld.hpp"

#include <functional>

#include "CircleCollider.hpp"

namespace
{
    [[nodiscard]] Guch2D::CollisionPoints
        CheckCollisionCircleCircle(const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                                   const std::shared_ptr<Guch2D::CollisionBody>& bodyB)
    {
        const auto& centerA = bodyA->GetColliderCenterWorld();
        const auto& centerB = bodyB->GetColliderCenterWorld();
        const auto& radiusA = std::dynamic_pointer_cast<Guch2D::CircleCollider>(
                                  bodyA->GetCollider())
                                  ->GetRadius();
        const auto& radiusB = std::dynamic_pointer_cast<Guch2D::CircleCollider>(
                                  bodyB->GetCollider())
                                  ->GetRadius();

        Guch2D::CollisionPoints collisionPoints;
        collisionPoints.HasCollision = Guch2D::VectLength(centerA - centerB) <= radiusA + radiusB;

        if (!collisionPoints.HasCollision) return collisionPoints;

        collisionPoints.A = centerA + Guch2D::VectNormalize(centerB - centerA) * radiusA;
        collisionPoints.B = centerB + Guch2D::VectNormalize(centerA - centerB) * radiusB;
        collisionPoints.Normal = Guch2D::VectNormalize(collisionPoints.B - collisionPoints.A);
        collisionPoints.Depth = Guch2D::VectLength(collisionPoints.B - collisionPoints.A);

        return collisionPoints;
    }
}   // namespace

namespace Guch2D
{
    void CollisionWorld::Step() const
    {
        ResolveCollisions();
    }

    void CollisionWorld::ResolveCollisions() const
    {
        std::vector<Collision> collisions;

        for (const auto& objectA : _objects)
        {
            for (const auto& objectB : _objects)
            {
                if (objectA == objectB) continue;

                if (const auto collisionPoints = CheckCollisions(objectA, objectB);
                    collisionPoints.HasCollision)
                {
                    collisions.emplace_back(objectA, objectB, collisionPoints);
                }
            }
        }
    }

    CollisionPoints CollisionWorld::CheckCollisions(const std::shared_ptr<CollisionBody>& bodyA,
                                                    const std::shared_ptr<CollisionBody>& bodyB)
    {
        constexpr auto typeCount = static_cast<size_t>(ColliderType::Count);

        using CollisionFunc = std::function<CollisionPoints(const std::shared_ptr<CollisionBody>&,
                                                            const std::shared_ptr<CollisionBody>&)>;
        using CollisionFuncMatrix = std::array<std::array<CollisionFunc, typeCount>, typeCount>;

        static const CollisionFuncMatrix collisionCheckMatrix = {
            {
             {nullptr, nullptr},                       // None
                {nullptr, &CheckCollisionCircleCircle},   // Circle
            }
        };

        if (!bodyA || !bodyB) return {};

        const auto& bodyACollider = bodyA->GetCollider();
        const auto& bodyBCollider = bodyB->GetCollider();

        if (!bodyACollider || !bodyBCollider) return {};

        const auto typeA = static_cast<size_t>(bodyACollider->GetColliderType());
        const auto typeB = static_cast<size_t>(bodyBCollider->GetColliderType());

        if (const auto& collisionFunc = collisionCheckMatrix.at(typeA).at(typeB); collisionFunc)
        {
            return collisionFunc(bodyA, bodyB);
        }

        return {};
    }
}   // namespace Guch2D
