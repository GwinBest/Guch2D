#pragma once

#include <memory>
#include <vector>

#include "Math/Vector.hpp"
#include "Objects/Object.hpp"

namespace Guch2D
{
    class PhysicsWorld final
    {
    public:
        void AddObject(const std::shared_ptr<Object>& object) { _objects.push_back(object); }

        void Step(const float deltaTime) noexcept;

    private:
        void ApplyGravity() noexcept;
        void UpdatePositions(const float deltaTime) noexcept;

    private:
        std::vector<std::shared_ptr<Object>> _objects;
        Vect _gravity = {0.0F, 9.81F};
    };
}   // namespace Guch2D