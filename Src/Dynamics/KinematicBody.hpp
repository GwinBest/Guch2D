#pragma once

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class KinematicBody final : public RigidBody
    {
    public:
        KinematicBody() noexcept = default;

        explicit KinematicBody(const Vect& position) noexcept
            : RigidBody(position)
        {}

        KinematicBody(const Vect& position, const float mass) noexcept
            : RigidBody(position, mass)
        {}

        KinematicBody(const KinematicBody&) = default;
        KinematicBody(KinematicBody&&) = default;
        KinematicBody& operator=(const KinematicBody&) = default;
        KinematicBody& operator=(KinematicBody&&) = default;
        ~KinematicBody() override = default;
        
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
