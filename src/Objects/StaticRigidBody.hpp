#pragma once

#include "Objects/Object.hpp"

namespace Guch2D
{
    class StaticRigidBody : public Object
    {
    public:
        StaticRigidBody() = default;
        StaticRigidBody(const StaticRigidBody&) = default;
        StaticRigidBody& operator=(const StaticRigidBody&) = default;
        StaticRigidBody(StaticRigidBody&&) = default;
        StaticRigidBody& operator=(StaticRigidBody&&) = default;
        ~StaticRigidBody() override = default;

    public:
        float Mass = 0.0F;
    };
}   // namespace Guch2D