#pragma once

#include "Collision/CollisionWorld.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
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

        void Step() const override;

        [[nodiscard]] const Vect& GetGravity() const noexcept { return _gravity; }

        void SetGravity(const Vect& gravity) noexcept
        {
            if (!IsFinite(gravity))
            {
                _gravity = DefaultGravity;
                return;
            }

            _gravity = gravity;
        }

    private:
        void ApplyGravity() const noexcept;

        void MoveBodies() const noexcept;

        void ApplyLinearDamping(
            const std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept;

    public:
        static constexpr Vect DefaultGravity = {0.0F, 9.81F};

    private:
        // Default Earth gravity is set by default
        Vect _gravity = DefaultGravity;
    };
}   // namespace Guch2D
