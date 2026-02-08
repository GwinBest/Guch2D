#pragma once

#include "Solver/Solver.hpp"

namespace Guch2D
{
    class PenetrationVectorSolver final : public Solver
    {
    public:
        PenetrationVectorSolver() = default;
        PenetrationVectorSolver(const PenetrationVectorSolver&) = default;
        PenetrationVectorSolver& operator=(const PenetrationVectorSolver&) = default;
        PenetrationVectorSolver(const PenetrationVectorSolver&&) = delete;
        PenetrationVectorSolver& operator=(const PenetrationVectorSolver&&) = delete;
        ~PenetrationVectorSolver() override = default;

        void Solve(const std::vector<Collision>& collisions) override;
    };
}   // namespace Guch2D
