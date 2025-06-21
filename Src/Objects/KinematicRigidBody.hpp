#pragma once

#include "Objects/RigidBody.hpp"

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

        void SetVelocity(const Vect& velocity) noexcept { _velocity = velocity; }

        [[nodiscard]] const Vect& GetVelocity() const noexcept { return _velocity; }

        void AddVelocity(const Vect& velocity) noexcept { _velocity += velocity; }

        void SetAcceleration(const Vect& acceleration) noexcept { _acceleration = acceleration; }

        [[nodiscard]] const Vect& GetAcceleration() const noexcept { return _acceleration; }

        private:
        // Velocity vector, in meters per second (m/s)
        Vect _velocity;

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration;
    };
}   // namespace Guch2D