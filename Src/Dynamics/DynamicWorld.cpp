#include "DynamicWorld.hpp"

#include "Dynamics/DynamicRigidBody.hpp"

namespace Guch2D
{
    void DynamicWorld::Step() const noexcept
    {
        ApplyGravity();
        MoveBodies();
    }

    constexpr void DynamicWorld::ApplyGravity() const noexcept
    {
        for (const auto& object : _objects)
        {
            if (const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object))
            {
                dynamicRigidBody->AddForce(dynamicRigidBody->GetMass() *
                                           (_gravity));
            }
        }
    }

    constexpr void DynamicWorld::MoveBodies() const noexcept
    {
        for (const auto& object : _objects)
        {
            if (const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object))
            {
                dynamicRigidBody->SetAcceleration(dynamicRigidBody->GetForce() /
                                                  dynamicRigidBody->GetMass());

                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep);

                dynamicRigidBody->UpdatePosition(dynamicRigidBody->GetVelocity() * _timeStep);

                // Reset force for the next step
                dynamicRigidBody->ResetForce();
            }
        }
    }
}
