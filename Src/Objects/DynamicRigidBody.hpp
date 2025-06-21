#pragma once

#include "Objects/RigidBody.hpp"

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

        void SetLinearDamping(const Vect& linearDamping) noexcept
        {
            _linearDamping = linearDamping;
        }

        [[nodiscard]] const Vect& GetLinearDamping() const noexcept { return _linearDamping; }

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