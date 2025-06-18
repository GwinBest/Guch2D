#include "PhysicsWorld.hpp"

#include "Objects/RigidBody.hpp"

namespace Guch2D
{
    void PhysicsWorld::Step(const float deltaTime) noexcept
    {
        ApplyGravity();
        UpdatePositions(deltaTime);
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

    void PhysicsWorld::UpdatePositions(const float deltaTime) noexcept
    {
        for (auto& object : _objects)
        {
            auto rigidBody = std::dynamic_pointer_cast<RigidBody>(object);
            if (rigidBody->GetType() != RigidBodyType::Static)
            {
                rigidBody->SetAcceleration(rigidBody->GetForce() / rigidBody->GetMass());
                rigidBody->SetVelocity(rigidBody->GetVelocity()
                                       + rigidBody->GetAcceleration() * deltaTime);

                rigidBody->SetPosition(rigidBody->GetPosition()
                                       + rigidBody->GetVelocity() * deltaTime);

                // Reset force for the next step
                rigidBody->ResetForce();
            }
        }
    }
}   // namespace Guch2D