#include "VelocitySolver.hpp"

#include <algorithm>
#include <cstdint>

#include "Collision/CollisionBody.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/RigidBody.hpp"

namespace
{
    void SingleVelocityIteration(const std::vector<Guch2D::Collision>& collisions)
    {
        for (const auto& [BodyA, BodyB, Points] : collisions)
        {
            const auto bodyA = BodyA.lock();
            const auto bodyB = BodyB.lock();

            if (!bodyA || !bodyB)
                continue;

            const auto rigidBodyA = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyA);
            const auto rigidBodyB = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyB);

            if (!rigidBodyA || !rigidBodyB)
                continue;

            const auto dynamicBodyA = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyA);
            const auto dynamicBodyB = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyB);

            const Guch2D::Vect velocityA = dynamicBodyA ? dynamicBodyA->GetVelocity()
                                                        : Guch2D::Vect {0.0F, 0.0F};
            const Guch2D::Vect velocityB = dynamicBodyB ? dynamicBodyB->GetVelocity()
                                                        : Guch2D::Vect {0.0F, 0.0F};

            const auto deltaVelocity = velocityB - velocityA;

            const auto velocityAlongNormal = Guch2D::VectDot(deltaVelocity, Points.Normal);

            // Collision normal is oriented from body B to body A.
            // With rv = (vB - vA), positive value means bodies are approaching along the
            // normal.
            if (velocityAlongNormal <= 0.0F)
                continue;

            const float invMassA = rigidBodyA->GetMass() == 0.0F ? 0.0F
                                                                 : 1.0F / rigidBodyA->GetMass();
            const float invMassB = rigidBodyB->GetMass() == 0.0F ? 0.0F
                                                                 : 1.0F / rigidBodyB->GetMass();

            const float invMassSum = invMassA + invMassB;

            if (invMassSum == 0.0F)
                continue;

            const float bounciness = std::min(rigidBodyA->GetBounciness(),
                                              rigidBodyB->GetBounciness());
            float impulseOfForce = (1.0F + bounciness) * velocityAlongNormal;
            impulseOfForce /= invMassSum;

            const Guch2D::Vect impulse = Points.Normal * impulseOfForce;

            if (dynamicBodyA)
            {
                dynamicBodyA->AddVelocity(impulse * invMassA);
            }

            if (dynamicBodyB)
            {
                dynamicBodyB->AddVelocity(-(impulse * invMassB));
            }
        }
    }
}   // namespace

namespace Guch2D
{
    void VelocitySolver::Solve(const std::vector<Collision>& collisions)
    {
        constexpr uint8_t totalIterations = 10;

        for (uint8_t iteration = 0; iteration < totalIterations; ++iteration)
        {
            SingleVelocityIteration(collisions);
        }
    }
}   // namespace Guch2D
