#pragma once

#include "Math/Vector.hpp"

namespace Guch2D
{
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

        constexpr auto SetCenter(const Vect& center) noexcept -> void { _center = center; }

        [[nodiscard]] constexpr auto GetCenter() const noexcept -> const Vect& { return _center; }

    protected:
        // Center of the collider in the 2D space, in meters (m)
        Vect _center;
    };
}   // namespace Guch2D