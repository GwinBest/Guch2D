#pragma once

#include <memory>

#include "Collision/Collider.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class CollisionBody
    {
    public:
        CollisionBody() noexcept = default;

        explicit CollisionBody(const Vect& position)
            : _position(position)
        {}

        explicit CollisionBody(const std::shared_ptr<Collider>& collider)
            : _collider(collider)
        {}

        CollisionBody(const Vect& position, const std::shared_ptr<Collider>& collider)
            : _position(position)
            , _collider(collider)
        {}

        CollisionBody(const CollisionBody&) = default;
        CollisionBody(CollisionBody&&) = default;
        CollisionBody& operator=(const CollisionBody&) = default;
        CollisionBody& operator=(CollisionBody&&) = default;
        virtual ~CollisionBody() = default;

        [[nodiscard]] constexpr const Vect& GetPosition() const noexcept { return _position; }

        constexpr void SetPosition(const Vect& position) noexcept { _position = position; }

        constexpr void UpdatePosition(const Vect& delta) noexcept { _position += delta; }

        [[nodiscard]] constexpr const std::shared_ptr<Collider>& GetCollider() const noexcept
        {
            return _collider;
        }

        constexpr void SetCollider(const std::shared_ptr<Collider>& collider) noexcept
        {
            _collider = collider;
        }

        [[nodiscard]] constexpr Vect GetColliderCenterWorld() const noexcept
        {
            if (!_collider) return _position;

            return _position + _collider->GetCenterLocal();
        }

    protected:
        // Position of the object in the 2D space, in meters (m)
        Vect _position = {0.0F, 0.0F};

        // Collider associated with this collision body
        std::shared_ptr<Collider> _collider;
    };
}   // namespace Guch2D
