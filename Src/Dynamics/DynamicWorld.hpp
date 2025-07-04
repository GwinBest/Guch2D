#pragma once

#include <memory>
#include <vector>

#include "Collisions/CollisionWorld.hpp"

namespace Guch2D
{
    // Forward declarations
    class DynamicRigidBody;

    class DynamicWorld : public CollisionWorld
    {
    public:
        DynamicWorld() = default;
        DynamicWorld(const DynamicWorld&) = default;
        DynamicWorld& operator=(const DynamicWorld&) = default;
        DynamicWorld(DynamicWorld&&) = default;
        DynamicWorld& operator=(DynamicWorld&&) = default;
        ~DynamicWorld() override = default;

        auto Step() -> void override;

        constexpr auto SetGravity(const Vect& gravity) noexcept { _gravity = gravity; }

        [[nodiscard]] constexpr auto GetGravity() const noexcept -> const Vect& { return _gravity; }

    private:
        auto ApplyGravity() noexcept -> void;
        auto MoveBodies() noexcept -> void;

        constexpr auto
            ApplyLinearDamping(std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept
            -> void;

    private:
        // Gravity vector, typically pointing downwards in the Y direction
        // Default value is 9.81F, which is the standard gravitational acceleration in m/s²
        Vect _gravity = {0.0F, 9.81F};
    };
}   // namespace Guch2D