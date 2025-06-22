#pragma once

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class DynamicRigidBody final : public RigidBody
    {
    public:
        DynamicRigidBody() = default;
        DynamicRigidBody(const DynamicRigidBody&) = default;
        DynamicRigidBody& operator=(const DynamicRigidBody&) = default;
        DynamicRigidBody(DynamicRigidBody&&) = default;
        DynamicRigidBody& operator=(DynamicRigidBody&&) = default;
        ~DynamicRigidBody() override = default;

        constexpr auto SetVelocity(const Vect& velocity) noexcept -> void { _velocity = velocity; }

        [[nodiscard]] constexpr auto GetVelocity() const noexcept -> const Vect&
        {
            return _velocity;
        }

        auto AddVelocity(const Vect& velocity) noexcept -> void { _velocity += velocity; }

        constexpr auto SetAcceleration(const Vect& acceleration) noexcept -> void
        {
            _acceleration = acceleration;
        }

        [[nodiscard]] constexpr auto GetAcceleration() const noexcept -> const Vect&
        {
            return _acceleration;
        }

        constexpr auto SetForce(const Vect& force) noexcept -> void { _force = force; }

        [[nodiscard]] constexpr auto GetForce() const noexcept -> const Vect& { return _force; }

        auto AddForce(const Vect& force) noexcept -> void { _force += force; }

        auto ResetForce() noexcept -> void { _force = {0.0F, 0.0F}; }

        constexpr auto SetGravityScale(const Vect& gravityScale) noexcept -> void
        {
            _gravityScale = gravityScale;
        }

        [[nodiscard]] constexpr auto GetGravityScale() const noexcept -> const Vect&
        {
            return _gravityScale;
        }

        constexpr auto SetLinearDamping(const Vect& linearDamping) noexcept -> void
        {
            _linearDamping = linearDamping;
        }

        [[nodiscard]] constexpr auto GetLinearDamping() const noexcept -> const Vect&
        {
            return _linearDamping;
        }

    private:
        // Velocity vector, in meters per second (m/s)
        Vect _velocity;

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration;

        // Force vector, in newtons (N)
        Vect _force;

        // Default gravity scale is 1.0F for both x and y axes
        Vect _gravityScale = {1.0F, 1.0F};

        // Linear damping vector, in meters per second squared (m/s²)
        // This is used to simulate air resistance
        // Default values are 0.0F for x and 0.1F for y
        Vect _linearDamping = {0.0F, 0.1F};
    };
}   // namespace Guch2D