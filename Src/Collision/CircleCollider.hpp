#pragma once

#include "Collision/Collider.hpp"

namespace Guch2D
{
    class CircleCollider final : public Collider
    {
    public:
        CircleCollider() noexcept
        {
            SetColliderType(ColliderType::Circle);
        }

        CircleCollider(const CircleCollider&) = default;
        CircleCollider(CircleCollider&&) = default;
        CircleCollider& operator=(const CircleCollider&) = default;
        CircleCollider& operator=(CircleCollider&&) = default;
        ~CircleCollider() override = default;

        [[nodiscard]] constexpr float GetRadius() const noexcept { return _radius; }

        constexpr void SetRadius(const float radius) noexcept
        {
            if (radius < 0.0F)
            {
                _radius = 0.0F;
                return;
            }

            _radius = radius;
        }

    private:
        // Radius of the circle collider, in meters (m)
        float _radius = 0.0F;
    };
}
