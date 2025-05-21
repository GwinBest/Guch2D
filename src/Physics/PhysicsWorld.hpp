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
        void AddObject(const std::shared_ptr<Object>& Object) { _Objects.push_back(Object); }

        void Step(const float DeltaTime);

    private:
        std::vector<std::shared_ptr<Object>> _Objects;
        Vect _Gravity = {0.0F, 9.81F};
    };
}   // namespace Guch2D