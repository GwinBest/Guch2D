#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "Collision/CollisionBody.hpp"
#include "Solver/PenetrationVectorSolver.hpp"

namespace Guch2D
{
    class Solver;

    class CollisionWorld
    {
    public:
        CollisionWorld() noexcept = default;
        CollisionWorld(const CollisionWorld&) = default;
        CollisionWorld(CollisionWorld&&) = default;
        CollisionWorld& operator=(const CollisionWorld&) = default;
        CollisionWorld& operator=(CollisionWorld&&) = default;
        virtual ~CollisionWorld() = default;

        virtual void Step() const;

        void AddObject(const std::shared_ptr<CollisionBody>& object)
        {
            if (!object) return;

            // Avoid duplicates
            if (std::ranges::find(_objects, object) == _objects.end())
            {
                _objects.push_back(object);
            }
        }

        void RemoveObject(const std::shared_ptr<CollisionBody>& object)
        {
            if (!object) return;

            std::erase(_objects, object);
        }

        void AddSolver(const std::shared_ptr<Solver>& solver)
        {
            if (!solver) return;

            // Avoid duplicates
            if (std::ranges::find(_solvers, solver) == _solvers.end())
            {
                _solvers.push_back(solver);
            }
        }

        void RemoveSolver(const std::shared_ptr<Solver>& solver)
        {
            if (!solver) return;

            std::erase(_solvers, solver);
        }

        [[nodiscard]] size_t GetObjectsCount() const noexcept { return _objects.size(); }

        [[nodiscard]] float GetTimeStep() const noexcept { return _timeStep; }

        void SetTimeStep(const float timeStep) noexcept
        {
            if (timeStep <= 0.0F || !std::isfinite(timeStep))
            {
                _timeStep = DefaultTimeStep;
                return;
            }

            _timeStep = timeStep;
        }

    protected:
        void FindCollisions() const;

        void SolveCollisions() const;

        void InvokeBeginOverlap() const;
        void InvokeEndOverlap() const;

        [[nodiscard]] static CollisionPoints CheckCollisions(std::shared_ptr<CollisionBody> bodyA,
                                                             std::shared_ptr<CollisionBody> bodyB);

    public:
        static constexpr float DefaultTimeStep = 1.0F / 60.0F;

    protected:
        std::vector<std::shared_ptr<CollisionBody>> _objects;
        std::vector<std::shared_ptr<Solver>> _solvers;

        float _timeStep = DefaultTimeStep;

        mutable std::vector<Collision> _collisions;
        mutable std::vector<Collision> _previousCollisions;
    };
}   // namespace Guch2D
