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

        void SetTimeStep(const float timeStep) noexcept
        {
            assert(timeStep > 0.0F);
            _timeStep = timeStep;
        }

        float GetTimeStep() const noexcept { return _timeStep; }

    private:
        void ApplyGravity() noexcept;
        void UpdatePositions() noexcept;

    private:
        std::vector<std::shared_ptr<Object>> _objects;

        Vect _gravity = {0.0F, 9.81F};

        // Default value for time step is 1/50 seconds
        float _timeStep = 1.0F / 50.0F;
    };
}   // namespace Guch2D