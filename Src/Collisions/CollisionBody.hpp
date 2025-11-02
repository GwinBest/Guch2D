#pragma once

#include <memory>

#include "Collisions/Collider.hpp"
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

        constexpr auto SetPosition(const Vect& position) noexcept -> void { _position = position; }

        [[nodiscard]] constexpr auto GetPosition() const noexcept -> const Vect&
        {
            return _position;
        }

        auto UpdatePosition(const Vect& delta) noexcept -> void { _position += delta; }

        auto SetCollider(const std::shared_ptr<Collider>& collider) noexcept -> void
        {
            _collider = collider;
        }

        [[nodiscard]] constexpr auto
            GetCollider() const noexcept -> const std::shared_ptr<Collider>&
        {
            return _collider;
        }

        [[nodiscard]] auto GetColliderCenterWorld() const noexcept -> Vect
        {
            if (_collider)
            {
                return _collider->GetCenterLocal() + _position;
            }

            return _position;
        }

    protected:
        // Position of the object in the 2D space, in meters (m)
        Vect _position = {0.0F, 0.0F};

        // Collider associated with this collision body
        std::shared_ptr<Collider> _collider;
    };
} // namespace Guch2D
