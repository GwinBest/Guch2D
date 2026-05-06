#include "DynamicWorld.hpp"

#include <cmath>
#include <print>

#include "Dynamics/DynamicRigidBody.hpp"

namespace Guch2D
{
    void DynamicWorld::Step()
    {
        CollisionWorld::Step();
        ApplyGravity();
        MoveBodies();
    }

    void DynamicWorld::ApplyGravity() const noexcept
    {
        for (const auto& object : _objects)
        {
            const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object);
            if (dynamicRigidBody && dynamicRigidBody->GetSimulatePhysics())
            {
                // Apply gravity scaled per-body
                dynamicRigidBody->AddForce(dynamicRigidBody->GetMass()
                                           * (dynamicRigidBody->GetGravityScale() * _gravity));
            }
        }
    }

    void DynamicWorld::MoveBodies() const noexcept
    {
        for (const auto& object : _objects)
        {
            const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object);
            if (dynamicRigidBody && dynamicRigidBody->GetSimulatePhysics())
            {
                const float mass = dynamicRigidBody->GetMass();
                if (mass == 0.0F)
                {
                    // Skip integration for zero mass
                    dynamicRigidBody->ResetForce();
                    continue;
                }

                dynamicRigidBody->SetAcceleration(dynamicRigidBody->GetForce() / mass);

                constexpr float HalfStepFactor = 0.5F;
                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep
                                              * HalfStepFactor);

                dynamicRigidBody->UpdatePosition(dynamicRigidBody->GetVelocity() * _timeStep);

                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep
                                              * HalfStepFactor);

                ApplyLinearDamping(dynamicRigidBody);

                // Reset force for the next step
                dynamicRigidBody->ResetForce();
            }
        }
    }

    void DynamicWorld::ApplyLinearDamping(
        const std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept
    {
        const float dampingFactorX = std::clamp(
            1.0F - (_timeStep * dynamicRigidBody->GetLinearDamping().x),
            0.0F,
            1.0F);
        const float dampingFactorY = std::clamp(
            1.0F - (_timeStep * dynamicRigidBody->GetLinearDamping().y),
            0.0F,
            1.0F);

        dynamicRigidBody->SetVelocity(dynamicRigidBody->GetVelocity()
                                      * Vect(dampingFactorX, dampingFactorY));
    }
}   // namespace Guch2D
