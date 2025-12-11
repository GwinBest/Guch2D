#pragma once

#include "Collision/CollisionWorld.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class DynamicWorld final : public CollisionWorld
    {
    public:
        DynamicWorld() noexcept = default;
        DynamicWorld(const DynamicWorld&) = default;
        DynamicWorld(DynamicWorld&&) = default;
        DynamicWorld& operator=(const DynamicWorld&) = default;
        DynamicWorld& operator=(DynamicWorld&&) = default;
        ~DynamicWorld() override = default;

        void Step() const noexcept override;

        [[nodiscard]] constexpr const Vect& GetGravity() const noexcept { return _gravity; }
        constexpr void SetGravity(const Vect& gravity) noexcept { _gravity = gravity; }

    private:
        constexpr void ApplyGravity() const noexcept;

        constexpr void MoveBodies() const noexcept;

    private:
        // Default Earth gravity is set by default
        Vect _gravity = {0.0F, 9.81F};
    };
}
