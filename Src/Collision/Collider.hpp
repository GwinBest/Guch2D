#pragma once

#include <cstdint>

#include "Math/Vector.hpp"

namespace Guch2D
{
    enum class ColliderType : std::uint8_t
    {
        None,
        Circle,
        Count   // To keep track of the number of collider types (keep always at the end)
    };

    class Collider
    {
    public:
        Collider() noexcept = default;

        explicit Collider(const Vect& center) { SetCenterLocal(center); }

        Collider(const Collider&) = default;
        Collider(Collider&&) = default;
        Collider& operator=(const Collider&) = default;
        Collider& operator=(Collider&&) = default;
        virtual ~Collider() = default;

        [[nodiscard]] constexpr ColliderType GetColliderType() const noexcept { return _type; }

        constexpr void SetColliderType(const ColliderType type) noexcept { _type = type; }

        [[nodiscard]] constexpr const Vect& GetCenterLocal() const noexcept { return _center; }

        constexpr void SetCenterLocal(const Vect& center) noexcept
        {
            if (!IsFinite(center))
            {
                _center = {0.0F, 0.0F};
                return;
            }

            _center = center;
        }

    private:
        ColliderType _type = ColliderType::None;

        // Center of the collider, in meters (m)
        // In local space
        Vect _center = {0.0F, 0.0F};
    };
}   // namespace Guch2D
