#pragma once

#include "Collision/Collider.hpp"

namespace Guch2D
{
    class AABBCollider final : public Collider
    {
    public:
        AABBCollider() noexcept { SetColliderType(ColliderType::AABB); }

        explicit AABBCollider(const Vect& extent) noexcept
        {
            SetColliderType(ColliderType::AABB);
            SetExtent(extent);
        }

        AABBCollider(const Vect& center,            // NOLINT(*-easily-swappable-parameters)
                     const Vect& extent) noexcept   // NOLINT(*-easily-swappable-parameters)
            : Collider(center)
        {
            SetColliderType(ColliderType::AABB);
            SetExtent(extent);
        }

        AABBCollider(const AABBCollider&) = default;
        AABBCollider(AABBCollider&&) = default;
        AABBCollider& operator=(const AABBCollider&) = default;
        AABBCollider& operator=(AABBCollider&&) = default;
        ~AABBCollider() override = default;

        void SetExtent(const Vect& extent) noexcept
        {
            if (!IsFinite(extent))
            {
                _extent = {0.0F, 0.0F};
                return;
            }

            _extent = extent;
        }

        [[nodiscard]] const Vect& GetExtent() const noexcept { return _extent; }

    private:
        // Extent of the collider in meters(m)
        Vect _extent = {0.0F, 0.0F};
    };
}   // namespace Guch2D
