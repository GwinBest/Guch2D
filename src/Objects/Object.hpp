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

        void SetPosition(const Vect& position) noexcept { _position = position; }

        const Vect& GetPosition() const noexcept { return _position; }

    protected:
        Vect _position = {0.0F, 0.0F};
    };
}   // namespace Guch2D