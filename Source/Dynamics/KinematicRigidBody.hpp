#pragma once

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class KinematicRigidBody final : public RigidBody
    {
    public:
        KinematicRigidBody() noexcept = default;

        explicit KinematicRigidBody(const Vect& position) noexcept
            : RigidBody(position)
        {}

        KinematicRigidBody(const KinematicRigidBody&) = default;
        KinematicRigidBody(KinematicRigidBody&&) = default;
        KinematicRigidBody& operator=(const KinematicRigidBody&) = default;
        KinematicRigidBody& operator=(KinematicRigidBody&&) = default;
        ~KinematicRigidBody() override = default;

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

        [[nodiscard]] const Vect& GetAngularAcceleration() const noexcept
        {
            return _angularAcceleration;
        }

        void SetAngularAcceleration(const Vect& acceleration) noexcept
        {
            if (!IsFinite(acceleration))
            {
                _angularAcceleration = {0.0F, 0.0F};
                return;
            }

            _angularAcceleration = acceleration;
        }

        [[nodiscard]] const Vect& GetAngularVelocity() const noexcept
        {
            return _angularVelocity;
        }

        void SetAngularVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
            {
                _angularVelocity = {0.0F, 0.0F};
                return;
            }

            _angularVelocity = velocity;
        }

        void AddAngularVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
                return;

            _angularVelocity += velocity;
        }

        [[nodiscard]] const Vect& GetAngularDamping() const noexcept { return _angularDamping; }

        void SetAngularDamping(const Vect& damping) noexcept
        {
            if (!IsFinite(damping))
            {
                _angularDamping = DefaultAngularDamping;
                return;
            }

            _angularDamping = damping;
        }

    public:
        static constexpr Vect DefaultAngularDamping = {0.1F, 0.1F};

    private:
        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration = {0.0F, 0.0F};

        // Velocity vector, in meters per second (m/s)
        Vect _velocity = {0.0F, 0.0F};

        // Angular velocity vector, in radian per second (rad/s)
        Vect _angularVelocity = {0.0F, 0.0F};

        // Angular acceleration vector, in radians per second squared (rad/s²)
        Vect _angularAcceleration = {0.0F, 0.0F};

        // Angular damping vector, in radians per second squared (rad/s²)
        // This is used to simulate air resistance
        // Default values are 0.1F for x and 0.1F for y
        Vect _angularDamping = DefaultAngularDamping;
    };
}   // namespace Guch2D
