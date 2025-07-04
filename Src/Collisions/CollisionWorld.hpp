#pragma once

#include <memory>
#include <vector>

#include "Collisions/CollisionBody.hpp"

namespace Guch2D
{
    class CollisionWorld
    {
    public:
        CollisionWorld() = default;
        CollisionWorld(const CollisionWorld&) = default;
        CollisionWorld& operator=(const CollisionWorld&) = default;
        CollisionWorld(CollisionWorld&&) = default;
        CollisionWorld& operator=(CollisionWorld&&) = default;
        virtual ~CollisionWorld() = default;

        constexpr auto AddObject(const std::shared_ptr<CollisionBody>& object) -> void
        {
            _objects.push_back(object);
        }

        virtual auto Step() -> void;

        auto ResolveCollisions() -> void;

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

        [[nodiscard]] constexpr auto GetTimeStep() const noexcept -> float { return _timeStep; }

    private:
        static auto CheckCollision(const std::shared_ptr<CollisionBody>& objectA,
                                   const std::shared_ptr<CollisionBody>& objectB) -> void;

    protected:
        std::vector<std::shared_ptr<CollisionBody>> _objects;

        // Default value for time step is 1/60 seconds
        float _timeStep = 1.0F / 60.0F;
    };
}   // namespace Guch2D