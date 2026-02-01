#pragma once

#include <cmath>
#include <glm/vec2.hpp>

namespace Guch2D
{
    using Vect = glm::vec2;

    [[nodiscard]] inline bool IsFinite(const float value) noexcept
    {
        return std::isfinite(value);
    }

    [[nodiscard]] inline bool IsFinite(const Vect& value) noexcept
    {
        return IsFinite(value.x) && IsFinite(value.y);
    }

    [[nodiscard]] inline float VectLength(const Vect& v) noexcept
    {
        if (!IsFinite(v))
        {
            return 0.0F;
        }

        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    [[nodiscard]] inline Vect VectNormalize(const Vect& v) noexcept
    {
        const float length = VectLength(v);
        if (length == 0.0F || !IsFinite(length))
        {
            return {0.0F, 0.0F};
        }

        return {v.x / length, v.y / length};
    }
}   // namespace Guch2D
