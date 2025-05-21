#pragma once

#include "Objects/Object.hpp"

namespace Guch2D
{
    class DynamicRigidBody : public Object
    {
    public:
        DynamicRigidBody() = default;
        DynamicRigidBody(const DynamicRigidBody&) = default;
        DynamicRigidBody& operator=(const DynamicRigidBody&) = default;
        DynamicRigidBody(DynamicRigidBody&&) = default;
        DynamicRigidBody& operator=(DynamicRigidBody&&) = default;
        ~DynamicRigidBody() override = default;

    public:
        float Mass = 0.0F;

        Vect Velocity;
        Vect Acceleration;
        Vect Force;
    };
}   // namespace Guch2D