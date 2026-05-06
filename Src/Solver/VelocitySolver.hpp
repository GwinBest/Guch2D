#pragma once

#include "Solver/Solver.hpp"

namespace Guch2D
{
    class VelocitySolver final : public Solver
    {
    public:
        VelocitySolver() = default;
        VelocitySolver(const VelocitySolver&) = default;
        VelocitySolver& operator=(const VelocitySolver&) = default;
        VelocitySolver(const VelocitySolver&&) = delete;
        VelocitySolver& operator=(const VelocitySolver&&) = delete;
        ~VelocitySolver() override = default;

        void Solve(const std::vector<Collision>& collisions) override;
    };
}   // namespace Guch2D
