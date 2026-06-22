#pragma once

#include "Solver/Solver.hpp"

namespace Guch2D
{
    class PositionSolver final : public Solver
    {
    public:
        PositionSolver() = default;
        PositionSolver(const PositionSolver&) = default;
        PositionSolver& operator=(const PositionSolver&) = default;
        PositionSolver(const PositionSolver&&) = delete;
        PositionSolver& operator=(const PositionSolver&&) = delete;
        ~PositionSolver() override = default;

        void Solve(const std::vector<Collision>& collisions) override;
    };
}   // namespace Guch2D
