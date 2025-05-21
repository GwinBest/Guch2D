#include "PhysicsWorld.hpp"

#include "Objects/DynamicRigidBody.hpp"

namespace Guch2D
{
    void PhysicsWorld::Step(const float DeltaTime)
    {
        for (auto& Object : _Objects)
        {
            if (auto* DynamicBody = dynamic_cast<DynamicRigidBody*>(Object.get()))
            {
                DynamicBody->Acceleration = _Gravity;
                DynamicBody->Velocity += DynamicBody->Acceleration * DeltaTime;
                DynamicBody->Position += DynamicBody->Velocity * DeltaTime
                                       + ((DynamicBody->Acceleration * (DeltaTime * DeltaTime))
                                          / 2.0F);

                // reset force at the end
                DynamicBody->Force = {0.0F, 0.0F};
            }
        }
    }
}   // namespace Guch2D