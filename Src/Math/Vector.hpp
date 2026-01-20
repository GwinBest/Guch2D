#pragma once

#include <cmath>
#include <glm/vec2.hpp>

namespace Guch2D
{
    using Vect = glm::vec2;

    [[nodiscard]] constexpr bool IsFinite(const float value) noexcept
    {
        return std::isfinite(value);
    }

    [[nodiscard]] constexpr bool IsFinite(const Vect& value) noexcept
    {
        return IsFinite(value.x) && IsFinite(value.y);
    }
}   // namespace Guch2D
