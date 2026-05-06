#include "Solver/PenetrationSolver.hpp"

#include "Collision/CollisionBody.hpp"
#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    void PenetrationSolver::Solve(const std::vector<Collision>& collisions)
    {
        for (const auto& [BodyA, BodyB, Points] : collisions)
        {
            const auto bodyA = BodyA.lock();
            const auto bodyB = BodyB.lock();

            if (!bodyA || !bodyB)
                continue;

            const auto rigidBodyA = std::dynamic_pointer_cast<RigidBody>(bodyA);
            const auto rigidBodyB = std::dynamic_pointer_cast<RigidBody>(bodyB);

            const float massA = rigidBodyA == nullptr ? 0.0F : rigidBodyA->GetMass();
            const float massB = rigidBodyB == nullptr ? 0.0F : rigidBodyB->GetMass();

            const float totalMass = massA + massB;

            if (totalMass == 0.0F)
                return;

            const Vect correctionA = Points.Normal * Points.Depth * (massB / totalMass);
            const Vect correctionB = Points.Normal * Points.Depth * (massA / totalMass);

            bodyA->UpdatePosition(correctionA);
            bodyB->UpdatePosition(-correctionB);
        }
    }
}   // namespace Guch2D
