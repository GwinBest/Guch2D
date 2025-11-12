#pragma once

#include <memory>
#include <vector>

#include "Collisions/CollisionWorld.hpp"

namespace Guch2D
{
    class DynamicRigidBody;

    class DynamicWorld final : public CollisionWorld
    {
    public:
        DynamicWorld() = default;
        DynamicWorld(const DynamicWorld&) = default;
        DynamicWorld& operator=(const DynamicWorld&) = default;
        DynamicWorld(DynamicWorld&&) = default;
        DynamicWorld& operator=(DynamicWorld&&) = default;
        ~DynamicWorld() override = default;

        auto Step() -> void override;

        constexpr auto SetGravity(const Vect& gravity) noexcept -> void { _gravity = gravity; }

        [[nodiscard]] constexpr auto GetGravity() const noexcept -> const Vect& { return _gravity; }

    private:
        auto ApplyGravity() const noexcept -> void;
        auto MoveBodies() const noexcept -> void;

        constexpr auto
            ApplyLinearDamping(
                const std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept
                -> void;

    private:
        // Gravity vector, typically pointing downwards in the Y direction
        // Default value is 9.81F, which is the standard gravitational acceleration in m/s²
        Vect _gravity = {0.0F, 9.81F};
    };
} // namespace Guch2D
