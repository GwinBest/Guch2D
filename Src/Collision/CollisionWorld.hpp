#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "Collision/CollisionBody.hpp"
#include "Solver/PenetrationSolver.hpp"

namespace Guch2D
{
    class Solver;

    enum class BroadPhaseType : std::uint8_t
    {
        SweepAndPrune,
        SpatialHashing,
    };

    class CollisionWorld
    {
    public:
        using ObjectType = std::shared_ptr<CollisionBody>;

    public:
        CollisionWorld() noexcept = default;
        CollisionWorld(const CollisionWorld&) = default;
        CollisionWorld(CollisionWorld&&) = default;
        CollisionWorld& operator=(const CollisionWorld&) = default;
        CollisionWorld& operator=(CollisionWorld&&) = default;
        virtual ~CollisionWorld() = default;

        virtual void Step();

        void AddObject(const ObjectType& object)
        {
            if (!object)
                return;

            // Avoid duplicates
            if (std::ranges::find(_objects, object) == _objects.end())
            {
                _objects.push_back(object);
            }
        }

        void RemoveObject(const ObjectType& object)
        {
            if (!object)
                return;

            std::erase(_objects, object);
        }

        void AddSolver(const std::shared_ptr<Solver>& solver)
        {
            if (!solver)
                return;

            // Avoid duplicates
            if (std::ranges::find(_solvers, solver) == _solvers.end())
            {
                _solvers.push_back(solver);
            }
        }

        void RemoveSolver(const std::shared_ptr<Solver>& solver)
        {
            if (!solver)
                return;

            std::erase(_solvers, solver);
        }

        [[nodiscard]] size_t GetObjectsCount() const noexcept { return _objects.size(); }

        [[nodiscard]] size_t GetSolversCount() const noexcept { return _solvers.size(); }

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

        [[nodiscard]] BroadPhaseType GetBroadPhaseType() const noexcept { return _broadPhaseType; }

        void SetBroadPhaseType(const BroadPhaseType broadPhaseType) noexcept
        {
            _broadPhaseType = broadPhaseType;
        }

    protected:
        std::vector<Collision> BroadPhase();

        void NarrowPhase(const std::vector<Collision>& possibleCollisions) const;

        void InvokeBeginOverlap() const;
        void InvokeEndOverlap() const;

        void SolveCollisions() const;

        [[nodiscard]] static CollisionPoints CheckCollisions(ObjectType bodyA, ObjectType bodyB);

    private:
        std::vector<Collision> SweepAndPrune();

        std::vector<Collision> SpatialHashing() const;

    public:
        static constexpr float DefaultTimeStep = 1.0F / 60.0F;

    protected:
        std::vector<ObjectType> _objects;
        std::vector<std::shared_ptr<Solver>> _solvers;

        float _timeStep = DefaultTimeStep;

        mutable std::vector<Collision> _collisions;
        mutable std::vector<Collision> _previousCollisions;

        BroadPhaseType _broadPhaseType = BroadPhaseType::SweepAndPrune;
    };
}   // namespace Guch2D
