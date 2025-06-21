#pragma once

#include "Math/Vector.hpp"

namespace Guch2D
{
    class CollisionBody
    {
    public:
        CollisionBody() = default;
        CollisionBody(const CollisionBody&) = default;
        CollisionBody& operator=(const CollisionBody&) = default;
        CollisionBody(CollisionBody&&) = default;
        CollisionBody& operator=(CollisionBody&&) = default;
        virtual ~CollisionBody() = default;

        void SetPosition(const Vect& position) noexcept { _position = position; }

        [[nodiscard]] const Vect& GetPosition() const noexcept { return _position; }

        void UpdatePosition(const Vect& delta) noexcept { _position += delta; }

    protected:
        // Position of the object in the 2D space, in meters (m)
        Vect _position = {0.0F, 0.0F};
    };
}   // namespace Guch2D