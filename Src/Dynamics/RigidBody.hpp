#pragma once

#include <cstdint>

#include "Collisions/CollisionBody.hpp"

namespace Guch2D
{
    class RigidBody : public CollisionBody
    {
    public:
        RigidBody() = default;
        RigidBody(const RigidBody&) = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody(RigidBody&&) = default;
        RigidBody& operator=(RigidBody&&) = default;
        ~RigidBody() override = default;

        constexpr auto SetMass(const float mass) noexcept -> void { _mass = mass; }

        [[nodiscard]] constexpr auto GetMass() const noexcept -> float { return _mass; }

    protected:
        // Mass of the rigid body, in kilograms (kg)
        float _mass = 0.0F;
    };
}   // namespace Guch2D