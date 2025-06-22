#pragma once

#include "Math/Vector.hpp"

namespace Guch2D
{
    enum ColliderType : std::uint8_t
    {
        None = 0,
        Circle,
        AABB,
    };

    class Collider
    {
    public:
        Collider() = default;

        explicit Collider(const Vect& center) noexcept
            : _center(center)
        {}

        Collider(const Collider&) = default;
        Collider& operator=(const Collider&) = default;
        Collider(Collider&&) = default;
        Collider& operator=(Collider&&) = default;
        virtual ~Collider() = default;

        constexpr auto SetCenterLocal(const Vect& center) noexcept -> void { _center = center; }

        [[nodiscard]] constexpr auto GetCenterLocal() const noexcept -> const Vect&
        {
            return _center;
        }

        [[nodiscard]] constexpr auto GetType() const noexcept -> ColliderType { return _type; }

    protected:
        // Center of the collider, in meters (m)
        // In local space
        Vect _center;

        ColliderType _type = ColliderType::None;
    };
}   // namespace Guch2D