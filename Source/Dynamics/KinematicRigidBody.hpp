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

    private:
        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration = {0.0F, 0.0F};

        // Velocity vector, in meters per second (m/s)
        Vect _velocity = {0.0F, 0.0F};
    };
}   // namespace Guch2D
