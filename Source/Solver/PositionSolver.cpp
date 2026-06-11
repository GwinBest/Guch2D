#include "Solver/PositionSolver.hpp"

#include <algorithm>
#include <cstdint>

#include "Collision/CollisionBody.hpp"
#include "Collision/CollisionWorld.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/RigidBody.hpp"

namespace
{
    constexpr float PenetrationSlop = 0.005F;
    constexpr float PositionalCorrectionPercent = 0.2F;
    constexpr float FullCorrectionDepthThreshold = 0.05F;

    [[nodiscard]] float GetInverseMass(const std::shared_ptr<Guch2D::RigidBody>& rigidBody)
    {
        if (!rigidBody)
            return 0.0F;

        if (const auto dynamicBody = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(rigidBody);
            dynamicBody && !dynamicBody->IsAwake())
        {
            return 0.0F;
        }

        const float mass = rigidBody->GetMass();
        return mass == 0.0F ? 0.0F : 1.0F / mass;
    }

    [[nodiscard]] Guch2D::CollisionPoints
        GetPositionSolvePoints(const Guch2D::Collision& collision,
                               const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                               const std::shared_ptr<Guch2D::CollisionBody>& bodyB,
                               const bool useCachedPoints)
    {
        if (useCachedPoints && collision.Points.HasCollision)
            return collision.Points;

        return Guch2D::CollisionWorld::CheckCollisions(bodyA, bodyB);
    }

    [[nodiscard]] float CalculateCorrectionDepth(const float depth) noexcept
    {
        if (depth <= 0.0F)
            return 0.0F;

        if (depth >= FullCorrectionDepthThreshold)
            return depth;

        return std::max(depth - PenetrationSlop, 0.0F) * PositionalCorrectionPercent;
    }
}   // namespace

namespace Guch2D
{
    void PositionSolver::Solve(const std::vector<Collision>& collisions)
    {
        constexpr std::uint8_t totalIterations = 8;

        for (std::uint8_t iteration = 0; iteration < totalIterations; ++iteration)
        {
            for (const auto& collision : collisions)
            {
                const auto bodyA = collision.BodyA.lock();
                const auto bodyB = collision.BodyB.lock();

                if (!bodyA || !bodyB)
                    continue;

                const auto points = GetPositionSolvePoints(collision, bodyA, bodyB, iteration == 0);

                if (!points.HasCollision)
                    continue;

                const float correctionDepth = CalculateCorrectionDepth(points.Depth);
                if (correctionDepth == 0.0F)
                    continue;

                const auto rigidBodyA = std::dynamic_pointer_cast<RigidBody>(bodyA);
                const auto rigidBodyB = std::dynamic_pointer_cast<RigidBody>(bodyB);

                const float invMassA = GetInverseMass(rigidBodyA);
                const float invMassB = GetInverseMass(rigidBodyB);
                const float invMassSum = invMassA + invMassB;

                if (invMassSum == 0.0F)
                    continue;

                const Vect correction = points.Normal * (correctionDepth / invMassSum);

                bodyA->UpdatePosition(correction * invMassA);
                bodyB->UpdatePosition(-(correction * invMassB));
            }
        }
    }
}   // namespace Guch2D
