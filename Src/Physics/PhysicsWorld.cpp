#include "PhysicsWorld.hpp"

#include "Objects/DynamicRigidBody.hpp"

namespace Guch2D
{
    void PhysicsWorld::Step() noexcept
    {
        ApplyGravity();
        MoveBodies();
    }

    void PhysicsWorld::ApplyGravity() noexcept
    {
        for (auto& object : _objects)
        {
            if (auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object))
            {
                dynamicRigidBody->AddForce(dynamicRigidBody->GetGravityScale() * _gravity
                                           * dynamicRigidBody->GetMass());
            }
        }
    }

    void PhysicsWorld::MoveBodies() noexcept
    {
        for (auto& object : _objects)
        {
            if (auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object))
            {
                dynamicRigidBody->SetAcceleration(dynamicRigidBody->GetForce()
                                                  / dynamicRigidBody->GetMass());

                // Apply half-step for velocity
                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep
                                              * 0.5F);

                dynamicRigidBody->UpdatePosition(dynamicRigidBody->GetVelocity() * _timeStep);

                // Apply another half-step for velocity
                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep
                                              * 0.5F);

                ApplyLinearDamping(dynamicRigidBody);

                // Reset force for the next step
                dynamicRigidBody->ResetForce();
            }
        }
    }

    void PhysicsWorld::ApplyLinearDamping(
        std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept
    {
        const auto dampingFactorX = std::clamp(
            1.0F - (_timeStep * dynamicRigidBody->GetLinearDamping().x),
            0.0F,
            1.0F);
        const auto dampingFactorY = std::clamp(
            1.0F - (_timeStep * dynamicRigidBody->GetLinearDamping().y),
            0.0F,
            1.0F);

        dynamicRigidBody->SetVelocity(dynamicRigidBody->GetVelocity()
                                      * Vect(dampingFactorX, dampingFactorY));
    }

}   // namespace Guch2D