#pragma once

#include <vector>

namespace Guch2D
{
    struct Collision;

    class Solver
    {
    public:
        Solver() = default;
        Solver(const Solver&) = default;
        Solver& operator=(const Solver&) = default;
        Solver(const Solver&&) = delete;
        Solver& operator=(const Solver&&) = delete;
        virtual ~Solver() = default;

        virtual void Solve(const std::vector<Collision>& collisions) = 0;
    };
}   // namespace Guch2D
