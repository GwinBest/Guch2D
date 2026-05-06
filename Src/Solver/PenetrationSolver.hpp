#pragma once

#include "Solver/Solver.hpp"

namespace Guch2D
{
    class PenetrationSolver final : public Solver
    {
    public:
        PenetrationSolver() = default;
        PenetrationSolver(const PenetrationSolver&) = default;
        PenetrationSolver& operator=(const PenetrationSolver&) = default;
        PenetrationSolver(const PenetrationSolver&&) = delete;
        PenetrationSolver& operator=(const PenetrationSolver&&) = delete;
        ~PenetrationSolver() override = default;

        void Solve(const std::vector<Collision>& collisions) override;
    };
}   // namespace Guch2D
