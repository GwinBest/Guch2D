#pragma once

#include "Math/Vector.hpp"
#include "Collision/CollisionBody.hpp"

namespace Guch2D
{
    class RigidBody : public CollisionBody
    {
    public:
        RigidBody() = default;

        explicit RigidBody(const Vect& position) : CollisionBody(position)
        {
        }

        RigidBody(const Vect& position, const float mass) : CollisionBody(position), _mass(mass)
        {
        }

        RigidBody(const RigidBody&) = default;
        RigidBody(RigidBody&&) = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody& operator=(RigidBody&&) = default;
        ~RigidBody() override = default;

        [[nodiscard]] constexpr float GetMass() const noexcept { return _mass; }
        constexpr void SetMass(const float mass) noexcept { _mass = mass; }

    protected:
        float _mass = 0.0F;
    };
}
