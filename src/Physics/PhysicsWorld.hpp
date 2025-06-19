#pragma once

#include <memory>
#include <vector>

#include "Math/Vector.hpp"
#include "Objects/Object.hpp"

namespace Guch2D
{
    class PhysicsWorld final
    {
    public:
        void AddObject(const std::shared_ptr<Object>& object) { _objects.push_back(object); }

        void Step() noexcept;

        // If timestep is less than or equal to zero, it will default to 1/60 seconds
        void SetTimeStep(const float timeStep) noexcept
        {
            if (timeStep <= 0.0F)
            {
                _timeStep = 1.0F / 60.0F;
                return;
            }

            _timeStep = timeStep;
        }

        void SetGravity(const Vect& gravity) noexcept { _gravity = gravity; }

        [[nodiscard]] const Vect& GetGravity() const noexcept { return _gravity; }

        [[nodiscard]] float GetTimeStep() const noexcept { return _timeStep; }

    private:
        void ApplyGravity() noexcept;
        void MoveBodies() noexcept;

    private:
        std::vector<std::shared_ptr<Object>> _objects;

        // Gravity vector, typically pointing downwards in the Y direction
        // Default value is 9.81F, which is the standard gravitational acceleration in m/s²
        Vect _gravity = {0.0F, 9.81F};

        // Default value for time step is 1/60 seconds
        float _timeStep = 1.0F / 60.0F;
    }
}   // namespace Guch2D