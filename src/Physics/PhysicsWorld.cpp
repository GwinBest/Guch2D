#include "PhysicsWorld.hpp"

#include "Objects/RigidBody.hpp"

namespace Guch2D
{
    void PhysicsWorld::Step() noexcept
    {
        ApplyGravity();
        UpdatePositions();
    }

    void PhysicsWorld::ApplyGravity() noexcept
    {
        for (auto& object : _objects)
        {
            auto rigidBody = std::dynamic_pointer_cast<RigidBody>(object);
            if (rigidBody->GetType() != RigidBodyType::Static)
            {
                rigidBody->AddForce(_gravity * rigidBody->GetMass());
            }
        }
    }

    void PhysicsWorld::UpdatePositions() noexcept
    {
        for (auto& object : _objects)
        {
            auto rigidBody = std::dynamic_pointer_cast<RigidBody>(object);
            if (rigidBody->GetType() != RigidBodyType::Static)
            {
                rigidBody->SetAcceleration(rigidBody->GetForce() / rigidBody->GetMass());
                rigidBody->SetVelocity(rigidBody->GetVelocity()
                                       + rigidBody->GetAcceleration() * _timeStep);

                rigidBody->SetPosition(rigidBody->GetPosition()
                                       + rigidBody->GetVelocity() * _timeStep);

                // Reset force for the next step
                rigidBody->ResetForce();
            }
        }
    }
}   // namespace Guch2D