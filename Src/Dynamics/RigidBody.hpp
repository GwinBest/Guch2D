#pragma once

#include "Collision/CollisionBody.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class RigidBody : public CollisionBody
    {
    public:
        RigidBody() noexcept = default;

        explicit RigidBody(const Vect& position) noexcept
            : CollisionBody(position)
        {}

        RigidBody(const Vect& position, const float mass) noexcept
            : CollisionBody(position)
            , _mass(mass)
        {}

        RigidBody(const RigidBody&) = default;
        RigidBody(RigidBody&&) = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody& operator=(RigidBody&&) = default;
        ~RigidBody() override = default;

        [[nodiscard]] constexpr float GetMass() const noexcept { return _mass; }

        constexpr void SetMass(const float mass) noexcept
        {
            if (mass <= 0.0F)
            {
                _mass = 0.0F;
                return;
            }

            _mass = mass;
        }

    private:
        float _mass = 0.0F;
    };
}   // namespace Guch2D
