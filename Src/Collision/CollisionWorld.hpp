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

        void AddObject(const std::shared_ptr<CollisionBody>& object) noexcept
        {
            if (!object) return;

            // Avoid duplicates
            if (std::ranges::find(_objects, object) == _objects.end())
            {
                _objects.push_back(object);
            }
        }

        void RemoveObject(const std::shared_ptr<CollisionBody>& object) noexcept
        {
            if (!object) return;
            std::erase(_objects, object);
        }

        [[nodiscard]] constexpr size_t GetObjectsCount() const noexcept { return _objects.size(); }

        [[nodiscard]] constexpr float GetTimeStep() const noexcept { return _timeStep; }

        void SetTimeStep(const float timeStep) noexcept
        {
            if (timeStep <= 0.0F)
            {
                _timeStep = 1.0F / 60.0F;
                return;
            }

            _timeStep = timeStep;
        }

    public:
        static constexpr float DefaultTimeStep = 1.0F / 60.0F;

    protected:
        std::vector<std::shared_ptr<CollisionBody>> _objects;

        float _timeStep = DefaultTimeStep;
    };
}   // namespace Guch2D
