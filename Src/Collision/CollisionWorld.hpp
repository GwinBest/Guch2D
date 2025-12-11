#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "CollisionBody.hpp"


namespace Guch2D
{
    class CollisionWorld
    {
    public:
        CollisionWorld() noexcept = default;
        CollisionWorld(const CollisionWorld&) = default;
        CollisionWorld(CollisionWorld&&) = default;
        CollisionWorld& operator=(const CollisionWorld&) = default;
        CollisionWorld& operator=(CollisionWorld&&) = default;
        virtual ~CollisionWorld() = default;

        virtual void Step() const noexcept;

        constexpr void AddObject(const std::shared_ptr<CollisionBody>& object) noexcept
        {
            _objects.push_back(object);
        }

        constexpr void RemoveObject(const std::shared_ptr<CollisionBody>& object) noexcept
        {
            const auto result = std::ranges::remove(_objects, object).begin();
            _objects.erase(result, _objects.end());
        }

        [[nodiscard]] constexpr float GetTimeStep() const noexcept { return _timeStep; }

        constexpr void SetTimeStep(const float timeStep) noexcept
        {
            if (timeStep <= 0.0F)
            {
                _timeStep = 1.0F / 60.0F;
                return;
            }

            _timeStep = timeStep;
        }

    protected:
        std::vector<std::shared_ptr<CollisionBody>> _objects;

        // Default time step is 1.0F / 60.0F
        float _timeStep = 1.0F / 60.0F;
    };
}
