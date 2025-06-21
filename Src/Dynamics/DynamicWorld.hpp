#pragma once

#include <memory>
#include <vector>

#include "Collisions/CollisionBody.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    // Forward declarations
    class DynamicRigidBody;

    class DynamicWorld final
    {
    public:
        constexpr auto AddObject(const std::shared_ptr<CollisionBody>& object)
        {
            _objects.push_back(object);
        }

        auto Step() noexcept -> void;

        // If timestep is less than or equal to zero, it will default to 1/60 seconds
        constexpr auto SetTimeStep(const float timeStep) noexcept
        {
            if (timeStep <= 0.0F)
            {
                _timeStep = 1.0F / 60.0F;
                return;
            }

            _timeStep = timeStep;
        }

        constexpr auto SetGravity(const Vect& gravity) noexcept { _gravity = gravity; }

        [[nodiscard]] constexpr auto GetGravity() const noexcept -> const Vect& { return _gravity; }

        [[nodiscard]] constexpr auto GetTimeStep() const noexcept -> float { return _timeStep; }

    private:
        auto ApplyGravity() noexcept -> void;
        auto MoveBodies() noexcept -> void;

        constexpr auto
            ApplyLinearDamping(std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept
            -> void;

    private:
        std::vector<std::shared_ptr<CollisionBody>> _objects;

        // Gravity vector, typically pointing downwards in the Y direction
        // Default value is 9.81F, which is the standard gravitational acceleration in m/s²
        Vect _gravity = {0.0F, 9.81F};

        // Default value for time step is 1/60 seconds
        float _timeStep = 1.0F / 60.0F;
    };
}   // namespace Guch2D