#pragma once

#include <limits>

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class StaticRigidBody final : public RigidBody
    {
    public:
        StaticRigidBody() noexcept
        {
            _mass = std::numeric_limits<float>::infinity();
        }

        explicit StaticRigidBody(const Vect& position) noexcept : RigidBody(position)
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
