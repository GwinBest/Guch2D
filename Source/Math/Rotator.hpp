#pragma once

#include <cmath>
#include <numbers>

namespace Guch2D
{
    using Rotator = float;

    [[nodiscard]] inline Rotator NormalizeRotator(const Rotator rotation) noexcept
    {
        constexpr float twoPi = std::numbers::pi_v<float> * 2.0F;

        const float newRotation = std::fmod(rotation, twoPi);
        if (!std::isfinite(newRotation))
        {
            return newRotation;
        }

        if (newRotation < 0.0F)
        {
            const float normalizedRotation = newRotation + twoPi;
            return normalizedRotation >= twoPi ? 0.0F : normalizedRotation;
        }

        return newRotation;
    }
}   // namespace Guch2D
