#pragma once

#include <vector>
#include <memory>
#include <algorithm>

#include "Math/Vector.hpp"

#include "Dynamics/RigidBody.hpp"

namespace Guch2D
{
    class DynamicWorld
    {
    public:
        void Step() const noexcept;

        [[nodiscard]] const Vect& GetGravity() const noexcept { return _gravity; }
        void SetGravity(const Vect& gravity) noexcept { _gravity = gravity; }

        void AddObject(const std::shared_ptr<RigidBody>& object) noexcept
        {
            _objects.push_back(object);
        }

        void RemoveObject(const std::shared_ptr<RigidBody>& object) noexcept
        {
            const auto result = std::ranges::remove(_objects, object).begin();
            _objects.erase(result, _objects.end());
        }

        [[nodiscard]] float GetTimeStep() const noexcept { return _timeStep; }

        void SetTimeStep(const float timeStep) noexcept
        {
            if (timeStep <= 0.0F)
            {
                _timeStep = 1.0F / 60.0F;
                return;
            }

            _timeStep = timeStep;
        }

    private:
        void ApplyGravity() const noexcept;

        void MoveBodies() const noexcept;

    private:
        // Default Earth gravity is set by default
        Vect _gravity = {0.0F, 9.91F};

        std::vector<std::shared_ptr<RigidBody>> _objects;

        // Default time step is 1.0F / 60.0F
        float _timeStep = 1.0F / 60.0F;
    };
}
