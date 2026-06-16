#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace Guch2D
{
    using Rotator = float;

    // Clamps an angle to the range [-pi, pi]
    [[nodiscard]] inline Rotator NormalizeRotator(const Rotator rotation) noexcept
    {
        constexpr float twoPi = std::numbers::pi_v<float> * 2.0F;

        float newRotation = std::fmod(rotation, twoPi);

        if (newRotation > std::numbers::pi_v<float>)
            newRotation -= twoPi;

        if (newRotation <= -std::numbers::pi_v<float>)
            newRotation += twoPi;

        return newRotation;
    }

    // Clamps an angle to the range [-180, 180]
    [[nodiscard]] inline Rotator NormalizeRotatorDegrees(const Rotator rotation) noexcept
    {
        constexpr float degToRad = std::numbers::pi_v<float> / 180.0F;
        constexpr float radToDeg = 180.0F / std::numbers::pi_v<float>;

        return NormalizeRotator(rotation * degToRad) * radToDeg;
    }
}   // namespace Guch2D
