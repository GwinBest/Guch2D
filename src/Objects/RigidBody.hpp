#pragma once

#include <cstdint>

#include "Objects/Object.hpp"

namespace Guch2D
{
    enum class RigidBodyType : std::uint8_t
    {
        Static = 0,
        Dynamic,
        Kinematic,
    };

    class RigidBody final : public Object
    {
    public:
        RigidBody() = default;
        RigidBody(const RigidBody&) = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody(RigidBody&&) = default;
        RigidBody& operator=(RigidBody&&) = default;
        ~RigidBody() override = default;

        void SetType(const RigidBodyType type) noexcept { _type = type; }

        RigidBodyType GetType() const noexcept { return _type; }

        void SetMass(const float mass) noexcept { _mass = mass; }

        float GetMass() const noexcept { return _mass; }

        void SetVelocity(const Vect& velocity) noexcept { _velocity = velocity; }

        const Vect& GetVelocity() const noexcept { return _velocity; }

        void SetAcceleration(const Vect& acceleration) noexcept { _acceleration = acceleration; }

        const Vect& GetAcceleration() const noexcept { return _acceleration; }

        void SetForce(const Vect& force) noexcept { _force = force; }

        const Vect& GetForce() const noexcept { return _force; }

        void AddForce(const Vect& force) noexcept { _force += force; }

        void ResetForce() noexcept { _force = {0.0F, 0.0F}; }

        void SetGravity(const Vect& gravity) noexcept { _gravity = gravity; }

        const Vect& GetGravity() const noexcept { return _gravity; }

    private:
        RigidBodyType _type = RigidBodyType::Static;

        float _mass = 0.0F;

        Vect _velocity;
        Vect _acceleration;
        Vect _force;
        Vect _gravity;
    };
}   // namespace Guch2D