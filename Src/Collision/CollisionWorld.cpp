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
#include "Solver/PenetrationVectorSolver.hpp"
#include "Utils/Logger.hpp"

namespace
{
    using SpatialBucket = std::vector<Guch2D::CollisionWorld::ObjectType>;
    using SpatialGridMap = std::unordered_map<size_t, SpatialBucket>;
    constexpr size_t SpatialHashingCalculateHash(int64_t cellX, int64_t cellY) noexcept;

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
                if (bucket.at(i) == bucket.at(j))
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
                if (objectA == objectB)
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

        collisionPoints.ContactPoints.front() = centerA + directionAB * radiusA;
        collisionPoints.ContactPoints.back() = centerB - directionAB * radiusB;
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
            const auto collisionPoints = CheckCollisions(collision.BodyA.lock(),
                                                         collision.BodyB.lock());

            if (!collisionPoints.HasCollision)
                continue;

            _collisions.emplace_back(collision.BodyA.lock(),
                                     collision.BodyB.lock(),
                                     collisionPoints);
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

        using CollisionFunc = std::function<CollisionPoints(const ObjectType&,
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
            WarnLog("Invalid collider type");
        }

        return {};
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
