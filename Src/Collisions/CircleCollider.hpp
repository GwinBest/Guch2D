#pragma once

#include "Collisions/Collider.hpp"

namespace Guch2D
{
    class CircleCollider final : public Collider
    {
    public:
        CircleCollider() = default;

        CircleCollider(const Vect& center, const float radius) noexcept
            : Collider(center)
              , _radius(radius)
        {
            _type = ColliderType::Circle;
        }

        CircleCollider(const CircleCollider&) = default;
        CircleCollider& operator=(const CircleCollider&) = default;
        CircleCollider(CircleCollider&&) = default;
        CircleCollider& operator=(CircleCollider&&) = default;
        ~CircleCollider() override = default;

        constexpr auto SetRadius(const float radius) noexcept -> void { _radius = radius; }

        [[nodiscard]] constexpr auto GetRadius() const noexcept -> float { return _radius; }

    private:
        // Radius of the circle collider, in meters (m)
        float _radius = 0.0F;
    };
} // namespace Guch2D
