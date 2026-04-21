#include "DynamicWorld.hpp"

#include <cmath>

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
            if (const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object))
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
            if (const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object))
            {
                const float mass = dynamicRigidBody->GetMass();
                if (mass == 0.0F)
                {
                    // Skip integration for zero mass
                    dynamicRigidBody->ResetForce();
                    continue;
                }

                dynamicRigidBody->SetAcceleration(dynamicRigidBody->GetForce() / mass);

                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep);

                dynamicRigidBody->UpdatePosition(dynamicRigidBody->GetVelocity() * _timeStep);

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
            1.0F
                - (_timeStep
                   * dynamicRigidBody->GetLinearDamping().x),   // NOLINT(*-pro-type-union-access)
            0.0F,
            1.0F);
        const float dampingFactorY = std::clamp(
            1.0F
                - (_timeStep
                   * dynamicRigidBody->GetLinearDamping().y),   // NOLINT(*-pro-type-union-access)
            0.0F,
            1.0F);

        dynamicRigidBody->SetVelocity(dynamicRigidBody->GetVelocity()
                                      * Vect(dampingFactorX, dampingFactorY));
    }
}   // namespace Guch2D
