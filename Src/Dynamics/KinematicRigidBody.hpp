#pragma once

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class KinematicRigidBody final : public RigidBody
    {
    public:
        KinematicRigidBody() = default;
        KinematicRigidBody(const KinematicRigidBody&) = default;
        KinematicRigidBody& operator=(const KinematicRigidBody&) = default;
        KinematicRigidBody(KinematicRigidBody&&) = default;
        KinematicRigidBody& operator=(KinematicRigidBody&&) = default;
        ~KinematicRigidBody() override = default;

        constexpr auto SetVelocity(const Vect& velocity) noexcept -> void { _velocity = velocity; }

        [[nodiscard]] constexpr auto GetVelocity() const noexcept -> const Vect&
        {
            return _velocity;
        }

        constexpr auto AddVelocity(const Vect& velocity) noexcept -> void { _velocity += velocity; }

        constexpr auto SetAcceleration(const Vect& acceleration) noexcept -> void
        {
            _acceleration = acceleration;
        }

        [[nodiscard]] constexpr auto GetAcceleration() const noexcept -> const Vect&
        {
            return _acceleration;
        }

    private:
        // Velocity vector, in meters per second (m/s)
        Vect _velocity;

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration;
    };
}   // namespace Guch2D