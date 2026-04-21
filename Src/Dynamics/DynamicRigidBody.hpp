#pragma once

#include "Dynamics/RigidBody.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class DynamicRigidBody final : public RigidBody
    {
    public:
        DynamicRigidBody() noexcept = default;

        explicit DynamicRigidBody(const Vect& position) noexcept
            : RigidBody(position)
        {}

        DynamicRigidBody(const Vect& position, const float mass) noexcept
            : RigidBody(position, mass)
        {}

        DynamicRigidBody(const DynamicRigidBody&) = default;
        DynamicRigidBody(DynamicRigidBody&&) = default;
        DynamicRigidBody& operator=(const DynamicRigidBody&) = default;
        DynamicRigidBody& operator=(DynamicRigidBody&&) = default;
        ~DynamicRigidBody() override = default;

        [[nodiscard]] const Vect& GetForce() const noexcept { return _force; }

        void SetForce(const Vect& force) noexcept
        {
            if (!IsFinite(force))
            {
                _force = {0.0F, 0.0F};
                return;
            }

            _force = force;
        }

        void AddForce(const Vect& force) noexcept
        {
            if (!IsFinite(force))
                return;

            _force += force;
        }

        void ResetForce() noexcept { _force = Vect(0.0F, 0.0F); }

        [[nodiscard]] const Vect& GetAcceleration() const noexcept { return _acceleration; }

        void SetAcceleration(const Vect& acceleration) noexcept
        {
            if (!IsFinite(acceleration))
            {
                _acceleration = {0.0F, 0.0F};
                return;
            }

            _acceleration = acceleration;
        }

        [[nodiscard]] const Vect& GetVelocity() const noexcept { return _velocity; }

        void SetVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
            {
                _velocity = {0.0F, 0.0F};
                return;
            }

            _velocity = velocity;
        }

        void AddVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
                return;

            _velocity += velocity;
        }

        [[nodiscard]] const Vect& GetGravityScale() const noexcept { return _gravityScale; }

        void SetGravityScale(const Vect& scale) noexcept
        {
            if (!IsFinite(scale))
            {
                _gravityScale = DefaultGravityScale;
                return;
            }

            _gravityScale = scale;
        }

        [[nodiscard]] const Vect& GetLinearDamping() const noexcept { return _linearDamping; }

        void SetLinearDamping(const Vect& damping) noexcept
        {
            if (!IsFinite(damping))
            {
                _linearDamping = DefaultLinearDamping;
                return;
            }

            _linearDamping = damping;
        }

    public:
        static constexpr Vect DefaultGravityScale = {1.0F, 1.0F};
        static constexpr Vect DefaultLinearDamping = {0.0F, 0.1F};

    private:
        // Force vector, in Newtons (N)
        Vect _force = {0.0F, 0.0F};

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration = {0.0F, 0.0F};

        // Velocity vector, in meters per second (m/s)
        Vect _velocity = {0.0F, 0.0F};

        // Default gravity scale is 1.0F for both x and y axes
        Vect _gravityScale = DefaultGravityScale;

        // Linear damping vector, in meters per second squared (m/s²)
        // This is used to simulate air resistance
        // Default values are 0.0F for x and 0.1F for y
        Vect _linearDamping = DefaultLinearDamping;
    };
}   // namespace Guch2D
