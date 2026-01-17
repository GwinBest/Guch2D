#include "DynamicWorld.hpp"

#include "Dynamics/DynamicRigidBody.hpp"
#include <cmath>

namespace Guch2D
{
    void DynamicWorld::Step() const noexcept
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
                dynamicRigidBody->AddForce(dynamicRigidBody->GetMass() *
                                           (dynamicRigidBody->GetGravityScale() * _gravity));
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
                if (mass == 0.0F || !std::isfinite(mass))
                {
                    // Skip integration for zero or infinite mass (static bodies)
                    dynamicRigidBody->ResetForce();
                    continue;
                }

                dynamicRigidBody->SetAcceleration(dynamicRigidBody->GetForce() /
                                                  mass);

                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep);

                dynamicRigidBody->UpdatePosition(dynamicRigidBody->GetVelocity() * _timeStep);

                // Reset force for the next step
                dynamicRigidBody->ResetForce();
            }
        }
    }
}
