#include "PhysicsWorld.hpp"

#include "Objects/RigidBody.hpp"

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
            auto rigidBody = std::dynamic_pointer_cast<RigidBody>(object);
            if (rigidBody->GetType() != RigidBodyType::Static)
            {
                rigidBody->AddForce(rigidBody->GetGravityScale() * _gravity * rigidBody->GetMass());
            }
        }
    }

    void PhysicsWorld::MoveBodies() noexcept
    {
        for (auto& object : _objects)
        {
            auto rigidBody = std::dynamic_pointer_cast<RigidBody>(object);
            if (rigidBody->GetType() != RigidBodyType::Static)
            {
                rigidBody->SetAcceleration(rigidBody->GetForce() / rigidBody->GetMass());

                // Apply half-step for velocity
                rigidBody->AddVelocity(rigidBody->GetAcceleration() * _timeStep * 0.5F);

                rigidBody->UpdatePosition(rigidBody->GetVelocity() * _timeStep);

                // Apply another half-step for velocity
                rigidBody->AddVelocity(rigidBody->GetAcceleration() * _timeStep * 0.5F);

                // Reset force for the next step
                rigidBody->ResetForce();
            }
        }
    }
}   // namespace Guch2D