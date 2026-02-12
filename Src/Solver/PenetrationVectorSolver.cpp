#include "Solver/PenetrationVectorSolver.hpp"

#include "Collision/CollisionBody.hpp"

namespace Guch2D
{
    void PenetrationVectorSolver::Solve(const std::vector<Collision>& collisions)
    {
        for (const auto& [BodyA, BodyB, Points] : collisions)
        {
            const auto bodyA = BodyA.lock();
            const auto bodyB = BodyB.lock();

            if (!bodyA || !bodyB) continue;

            const Vect correction = Points.Normal * (Points.Depth / 2.0F);

            bodyA->UpdatePosition(correction);
            bodyB->UpdatePosition(-correction);
        }
    }
}   // namespace Guch2D
