#pragma once

#include <algorithm>

#include "Collision/CollisionBody.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class RigidBody : public CollisionBody
    {
    private:
        enum class MassMode : std::uint8_t
        {
            Manual,
            FromDensity
        };

    public:
        RigidBody() noexcept = default;

        explicit RigidBody(const Vect& position) noexcept
            : CollisionBody(position)
        {}

        RigidBody(const Vect& position, const float mass) noexcept
            : CollisionBody(position)
        {
            SetMass(mass);
        }

        RigidBody(const RigidBody&) = default;
        RigidBody(RigidBody&&) = default;
        RigidBody& operator=(const RigidBody&) = default;
        RigidBody& operator=(RigidBody&&) = default;
        ~RigidBody() override = 0;

        [[nodiscard]] float GetMass() const noexcept
        {
            if (_massMode == MassMode::FromDensity)
            {
                const auto collider = GetCollider();
                if (!collider)
                    return 0.0F;

                const float mass = _density * collider->GetArea();
                return (mass > 0.0F && IsFinite(mass)) ? mass : 0.0F;
            }

            return _mass;
        }

        void SetMass(const float mass) noexcept
        {
            _massMode = MassMode::Manual;
            _density = 0.0F;
            _mass = (mass > 0.0F && IsFinite(mass)) ? mass : 0.0F;
        }

        [[nodiscard]] float GetDensity() const noexcept { return _density; }

        void SetDensity(const float density) noexcept
        {
            _density = (density > 0.0F && IsFinite(density)) ? density : 0.0F;
            _massMode = (_density > 0.0F) ? MassMode::FromDensity : MassMode::Manual;
        }

        [[nodiscard]] float GetBounciness() const noexcept { return _bounciness; }

        // Bounciness always stays between 0 and 1
        // A value of 0 indicates no bounce while a value of 1 indicates a perfect bounce with no
        // loss of energy.
        void SetBounciness(const float bounciness) noexcept
        {
            if (!IsFinite(bounciness))
            {
                _bounciness = 0.0F;
                return;
            }

            _bounciness = std::clamp(bounciness, 0.0F, 1.0F);
        }

    private:
        // Mass of the rigid body, in kilograms (kg)
        // NOTE: Do not access this field directly, use GetMass() instead
        float _mass = 0.0F;

        // Density of the rigid body, in kilograms per square meter (kg/m^2)
        float _density = 0.0F;

        // Bounciness of the rigid body
        // Bounciness always stays between 0 and 1
        // A value of 0 indicates no bounce while a value of 1 indicates a perfect bounce with no
        // loss of energy (if friction and linear damping is 0).
        float _bounciness = 0.0F;

        MassMode _massMode = MassMode::Manual;
    };

    inline RigidBody::~RigidBody() = default;
}   // namespace Guch2D
