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
        const auto delta = centerB - centerA;
        const float distance = Guch2D::VectLength(delta);
        const float radiusSum = radiusA + radiusB;

        collisionPoints.HasCollision = distance <= radiusSum;

        if (!collisionPoints.HasCollision) return collisionPoints;

        const Guch2D::Vect directionAB = distance > 0.0F ? delta / distance
                                                         : Guch2D::Vect {1.0F, 0.0F};

        collisionPoints.A = centerA + directionAB * radiusA;
        collisionPoints.B = centerB - directionAB * radiusB;
        collisionPoints.Normal = -directionAB;
        collisionPoints.Depth = radiusSum - distance;

        return collisionPoints;
    }

    [[nodiscard]] bool HasSameOverlapPair(const Guch2D::Collision& lhs,
                                          const Guch2D::Collision& rhs)
    {
        const auto lhsA = lhs.BodyA.lock();
        const auto lhsB = lhs.BodyB.lock();
        const auto rhsA = rhs.BodyA.lock();
        const auto rhsB = rhs.BodyB.lock();

        if (!lhsA || !lhsB || !rhsA || !rhsB) return false;

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
                if (objectA == objectB) break;

                const auto collisionPoints = CheckCollisions(objectA, objectB);
                if (!collisionPoints.HasCollision) continue;

                const auto collision = Collision(objectA, objectB, collisionPoints);
                _collisions.emplace_back(collision);
            }
        }
    }

    void CollisionWorld::SolveCollisions() const
    {
        for (const auto& solver : _solvers)
        {
            if (solver) solver->Solve(_collisions);
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

            if (bodyA) bodyA->InvokeOnBeginOverlap(collision);
            if (bodyB) bodyB->InvokeOnBeginOverlap(collision);
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

        static const CollisionFuncMatrix CollisionCheckMatrix = {
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
