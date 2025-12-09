#pragma once

#include "Math/Vector.hpp"

namespace Guch2D
{
    class RigidBody
    {
    public:
        RigidBody() = default;

        explicit RigidBody(const Vect& position) : _position(position)
        {
        }

        RigidBody(const Vect& position, const float mass) : _position(position), _mass(mass)
        {
        }

        RigidBody(const RigidBody&) = default;
        RigidBody(RigidBody&&) = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody& operator=(RigidBody&&) = default;
        virtual ~RigidBody() = default;

        [[nodiscard]] Vect GetPosition() const noexcept { return _position; }
        void SetPosition(const Vect& position) noexcept { _position = position; }
        void UpdatePosition(const Vect& position) noexcept { _position += position; }

        [[nodiscard]] float GetMass() const noexcept { return _mass; }
        void SetMass(const float mass) noexcept { _mass = mass; }

    protected:
        Vect _position = {0.0F, 0.0F};

        float _mass = 0.0F;
    };
}
