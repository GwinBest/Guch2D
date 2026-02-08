#include "CollisionWorld.hpp"

#include <array>
#include <functional>
#include <print>

#include "Collision/CircleCollider.hpp"
#include "Solver/PenetrationVectorSolver.hpp"

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
                if (objectA == objectB) break;

                const auto collisionPoints = CheckCollisions(objectA, objectB);
                if (!collisionPoints.HasCollision) continue;

                const auto collision = Collision(objectA, objectB, collisionPoints);
                _collisions.emplace_back(collision);
            }
        }
    }

    void CollisionWorld::InvokeBeginOverlap() const
    {
        // Invoke OnBeginOverlap if this collision was not present in the previous frame
        std::ranges::for_each(_collisions, [&](const Collision& collision) {
            if (std::ranges::find(_previousCollisions, collision) != _previousCollisions.end())
            {
                return;
            }

            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();

            if (bodyA) bodyA->InvokeOnBeginOverlap(collision);
            if (bodyB) bodyB->InvokeOnBeginOverlap(collision);
        });
    }

    void CollisionWorld::InvokeEndOverlap() const
    {
        // Invoke OnEndOverlap if this collision was present in the previous frame
        std::ranges::for_each(_previousCollisions, [&](const Collision& collision) {
            if (std::ranges::find(_collisions, collision) != _collisions.end()) return;

            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();

            if (bodyA) bodyA->InvokeOnEndOverlap(collision);
            if (bodyB) bodyB->InvokeOnEndOverlap(collision);
        });
    }

    CollisionPoints CollisionWorld::CheckCollisions(std::shared_ptr<CollisionBody> bodyA,
                                                    std::shared_ptr<CollisionBody> bodyB)
    {
        constexpr auto typeCount = static_cast<size_t>(ColliderType::Count);

        using CollisionFunc = std::function<CollisionPoints(const std::shared_ptr<CollisionBody>&,
                                                            const std::shared_ptr<CollisionBody>&)>;
        using CollisionFuncMatrix = std::array<std::array<CollisionFunc, typeCount>, typeCount>;

        static const CollisionFuncMatrix collisionCheckMatrix = {
            {
             //     None          Circle
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

        const bool swap = typeB > typeA;
        if (swap)
        {
            std::swap(bodyA, bodyB);
        }

        try
        {
            if (const auto& collisionFunc = collisionCheckMatrix.at(typeA).at(typeB); collisionFunc)
            {
                auto collisionPoints = collisionFunc(bodyA, bodyB);

                if (swap)
                {
                    std::swap(collisionPoints.A, collisionPoints.B);
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
