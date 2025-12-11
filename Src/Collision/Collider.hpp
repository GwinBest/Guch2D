#pragma once

#include <cstdint>

#include "Math/Vector.hpp"

namespace Guch2D
{
    enum ColliderType : std::uint8_t
    {
        None,
        Circle,
    };

    class Collider
    {
    public:
        Collider() noexcept = default;

        explicit Collider(const Vect& center) : _center(center)
        {
        }

        Collider(const Collider&) = default;
        Collider(Collider&&) = default;
        Collider& operator=(const Collider&) = default;
        Collider& operator=(Collider&&) = default;
        virtual ~Collider() = default;

        [[nodiscard]] constexpr ColliderType GetColliderType() const noexcept { return _type; }
        constexpr void SetColliderType(const ColliderType type) noexcept { _type = type; }

        [[nodiscard]] constexpr const Vect& GetCenterLocal() const noexcept { return _center; }
        constexpr void SetCenterLocal(const Vect& center) noexcept { _center = center; }

    private:
        ColliderType _type = None;

        // Center of the collider, in meters (m)
        // In local space
        Vect _center = {0.0F, 0.0F};
    };
}
