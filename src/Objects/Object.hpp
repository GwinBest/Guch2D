#pragma once

#include "Math/Vector.hpp"

namespace Guch2D
{
    class Object
    {
    public:
        Object() = default;
        Object(const Object&) = default;
        Object& operator=(const Object&) = default;
        Object(Object&&) = default;
        Object& operator=(Object&&) = default;
        virtual ~Object() = default;

    public:
        Vect Position = {0.0F, 0.0F};
    };
}   // namespace Guch2D