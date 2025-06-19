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

        [[nodiscard]] RigidBodyType GetType() const noexcept { return _type; }

        void SetMass(const float mass) noexcept { _mass = mass; }

        [[nodiscard]] float GetMass() const noexcept { return _mass; }

        void SetVelocity(const Vect& velocity) noexcept { _velocity = velocity; }

        [[nodiscard]] const Vect& GetVelocity() const noexcept { return _velocity; }

        void AddVelocity(const Vect& velocity) noexcept { _velocity += velocity; }

        void SetAcceleration(const Vect& acceleration) noexcept { _acceleration = acceleration; }

        [[nodiscard]] const Vect& GetAcceleration() const noexcept { return _acceleration; }

        void SetForce(const Vect& force) noexcept { _force = force; }

        [[nodiscard]] const Vect& GetForce() const noexcept { return _force; }

        void AddForce(const Vect& force) noexcept { _force += force; }

        void ResetForce() noexcept { _force = {0.0F, 0.0F}; }

        void SetGravityScale(const Vect& gravityScale) noexcept { _gravityScale = gravityScale; }

        [[nodiscard]] const Vect& GetGravityScale() const noexcept { return _gravityScale; }

    private:
        RigidBodyType _type = RigidBodyType::Static;

        // Mass of the rigid body, in kilograms (kg)
        float _mass = 0.0F;

        // Velocity vector, in meters per second (m/s)
        Vect _velocity;

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration;

        // Force vector, in newtons (N)
        Vect _force;

        // Default gravity scale is 1.0F for both x and y axes
        Vect _gravityScale = {1.0F, 1.0F};

        Vect _linearDamping;
    };
}   // namespace Guch2D