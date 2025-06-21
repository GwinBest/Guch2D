#pragma once

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class StaticRigidBody final : public RigidBody
    {
    public:
        StaticRigidBody() = default;
        StaticRigidBody(const StaticRigidBody&) = default;
        StaticRigidBody& operator=(const StaticRigidBody&) = default;
        StaticRigidBody(StaticRigidBody&&) = default;
        StaticRigidBody& operator=(StaticRigidBody&&) = default;
        ~StaticRigidBody() override = default;
    };
}   // namespace Guch2D