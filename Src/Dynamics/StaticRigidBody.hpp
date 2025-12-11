#pragma once

#include <numeric>

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class StaticRigidBody final : public RigidBody
    {
    public:
        StaticRigidBody()
        {
            _mass = std::numeric_limits<float>::infinity();
        }

        explicit StaticRigidBody(const Vect& position) : RigidBody(position)
        {
            _mass = std::numeric_limits<float>::infinity();
        }

        StaticRigidBody(const StaticRigidBody&) = default;
        StaticRigidBody(StaticRigidBody&&) = default;
        StaticRigidBody& operator=(const StaticRigidBody&) = default;
        StaticRigidBody& operator=(StaticRigidBody&&) = default;
        ~StaticRigidBody() override = default;
    };
}
