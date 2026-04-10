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
        return IsFinite(value.x) && IsFinite(value.y);   // NOLINT(*-pro-type-union-access)
    }

    [[nodiscard]] inline float VectLength(const Vect& vect) noexcept
    {
        if (!IsFinite(vect))
        {
            return 0.0F;
        }

        return std::sqrt((vect.x * vect.x)
                         + (vect.y * vect.y));   // NOLINT(*-pro-type-union-access)
    }

    [[nodiscard]] inline Vect VectNormalize(const Vect& vect) noexcept
    {
        const float length = VectLength(vect);
        if (length == 0.0F || !IsFinite(length))
        {
            return {0.0F, 0.0F};
        }

        return {vect.x / length, vect.y / length};   // NOLINT(*-pro-type-union-access)
    }
}   // namespace Guch2D
