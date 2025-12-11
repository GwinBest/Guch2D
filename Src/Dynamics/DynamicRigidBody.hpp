#pragma once

#include "Dynamics/RigidBody.hpp"

#include "Math/Vector.hpp"

namespace Guch2D
{
    class DynamicRigidBody final : public RigidBody
    {
    public:
        DynamicRigidBody() = default;

        explicit DynamicRigidBody(const Vect& position) : RigidBody(position)
        {
        }

        DynamicRigidBody(const Vect& position, const float mass) : RigidBody(position, mass)
        {
        }

        DynamicRigidBody(const DynamicRigidBody&) = default;
        DynamicRigidBody(DynamicRigidBody&&) = default;
        DynamicRigidBody& operator=(const DynamicRigidBody&) = default;
        DynamicRigidBody& operator=(DynamicRigidBody&&) = default;
        ~DynamicRigidBody() override = default;

        [[nodiscard]] constexpr const Vect& GetForce() const noexcept { return _force; }
        constexpr void SetForce(const Vect& force) noexcept { _force = force; }
        constexpr void AddForce(const Vect& force) noexcept { _force += force; }
        constexpr void ResetForce() noexcept { _force = Vect(0.0F, 0.0F); }

        [[nodiscard]] constexpr const Vect& GetAcceleration() const noexcept
        {
            return _acceleration;
        }

        constexpr void SetAcceleration(const Vect& acceleration) noexcept
        {
            _acceleration = acceleration;
        }

        [[nodiscard]] constexpr const Vect& GetVelocity() const noexcept { return _velocity; }
        constexpr void SetVelocity(const Vect& velocity) noexcept { _velocity = velocity; }
        constexpr void AddVelocity(const Vect& velocity) noexcept { _velocity += velocity; }

        [[nodiscard]] constexpr const Vect& GetGravityScale() const noexcept
        {
            return _gravityScale;
        }

        constexpr void SetGravityScale(const Vect& scale) noexcept { _gravityScale = scale; }

        [[nodiscard]] constexpr const Vect& GetLinearDamping() const noexcept
        {
            return _linearDamping;
        }

        constexpr void SetLinearDamping(const Vect& damping) noexcept { _linearDamping = damping; }

    private:
        // Force vector, in Newtons (N)
        Vect _force = {0.0F, 0.0F};

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration = {0.0F, 0.0F};

        // Velocity vector, in meters per second (m/s)
        Vect _velocity = {0.0F, 0.0F};

        // Default gravity scale is 1.0F for both x and y axes
        Vect _gravityScale = {0.0F, 0.0F};

        // Linear damping vector, in meters per second squared (m/s²)
        // This is used to simulate air resistance
        // Default values are 0.0F for x and 0.1F for y
        Vect _linearDamping = {0.0F, 0.1F};
    };
}
