#pragma once

#include <limits>

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class StaticRigidBody final : public RigidBody
    {
    public:
        StaticRigidBody() noexcept { SetMass(DefaultStaticBodyMass); }

        explicit StaticRigidBody(const Vect& position) noexcept
            : RigidBody(position)
        {
            SetMass(DefaultStaticBodyMass);
        }

        StaticRigidBody(const StaticRigidBody&) = default;
        StaticRigidBody(StaticRigidBody&&) = default;
        StaticRigidBody& operator=(const StaticRigidBody&) = default;
        StaticRigidBody& operator=(StaticRigidBody&&) = default;
        ~StaticRigidBody() override = default;

    public:
        static constexpr float DefaultStaticBodyMass = 0.0F;
    };
}   // namespace Guch2D
