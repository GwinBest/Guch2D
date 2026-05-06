#include "VelocitySolver.hpp"

#include <cstdint>

#include "Collision/CollisionBody.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    void VelocitySolver::Solve(const std::vector<Collision>& collisions)
    {
        constexpr uint8_t totalIterations = 10;

        for (uint8_t iteration = 0; iteration < totalIterations; ++iteration)
        {
            for (const auto& [BodyA, BodyB, Points] : collisions)
            {
                const auto bodyA = BodyA.lock();
                const auto bodyB = BodyB.lock();

                if (!bodyA || !bodyB)
                    continue;

                const auto rigidBodyA = std::dynamic_pointer_cast<RigidBody>(bodyA);
                const auto rigidBodyB = std::dynamic_pointer_cast<RigidBody>(bodyB);

                if (!rigidBodyA || !rigidBodyB)
                    continue;

                const auto dynamicBodyA = std::dynamic_pointer_cast<DynamicRigidBody>(bodyA);
                const auto dynamicBodyB = std::dynamic_pointer_cast<DynamicRigidBody>(bodyB);

                const Vect velocityA = dynamicBodyA ? dynamicBodyA->GetVelocity()
                                                    : Vect {0.0F, 0.0F};
                const Vect velocityB = dynamicBodyB ? dynamicBodyB->GetVelocity()
                                                    : Vect {0.0F, 0.0F};

                const auto deltaVelocity = velocityB - velocityA;

                const auto velocityAlongNormal = VectDot(deltaVelocity, Points.Normal);

                // Collision normal is oriented from body B to body A.
                // With rv = (vB - vA), positive value means bodies are approaching along the normal.
                if (velocityAlongNormal <= 0.0F)
                    continue;

                const float invMassA = rigidBodyA->GetMass() == 0.0F ? 0.0F
                                                                     : 1.0F / rigidBodyA->GetMass();
                const float invMassB = rigidBodyB->GetMass() == 0.0F ? 0.0F
                                                                     : 1.0F / rigidBodyB->GetMass();

                const float invMassSum = invMassA + invMassB;

                if (invMassSum == 0.0F)
                    continue;

                float impulseOfForce = (1.0F + 0.0F) * velocityAlongNormal;
                impulseOfForce /= invMassSum;

                const Vect impulse = Points.Normal * impulseOfForce;

                if (dynamicBodyA && rigidBodyA->GetMass() > 0.0F)
                {
                    dynamicBodyA->AddVelocity(impulse * invMassA);
                }
                if (dynamicBodyB && rigidBodyB->GetMass() > 0.0F)
                {
                    dynamicBodyB->AddVelocity(-(impulse * invMassB));
                }
            }
        }
    }
}   // namespace Guch2D
