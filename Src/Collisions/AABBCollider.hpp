#pragma once

#include "Collisions/Collider.hpp"

namespace Guch2D
{
    class AABBCollider final : public Collider
    {
    public:
        AABBCollider() = default;

        AABBCollider(const Vect& center, const Vect& extends) noexcept
            : Collider(center)
            , _extends(extends)
        {
            _type = ColliderType::AABB;
        }

        AABBCollider(const AABBCollider&) = default;
        AABBCollider& operator=(const AABBCollider&) = default;
        AABBCollider(AABBCollider&&) = default;
        AABBCollider& operator=(AABBCollider&&) = default;
        ~AABBCollider() override = default;

        constexpr auto SetExtends(const Vect& extends) noexcept -> void { _extends = extends; }

        [[nodiscard]] constexpr auto GetExtends() const noexcept -> const Vect& { return _extends; }

    private:
        // Half extent of the line collider, in meters (m)
        Vect _extends;
    };
}   // namespace Guch2D