#pragma once

#include <functional>
#include <memory>

#include "Collision/Collider.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class CollisionBody;
    class CollisionWorld;

    struct CollisionPoints final
    {
        Vect A = {0.0F, 0.0F};        // Furthest point of A into B
        Vect B = {0.0F, 0.0F};        // Furthest point of B into A
        Vect Normal = {0.0F, 0.0F};   // B – A normalized
        float Depth = 0.0F;           // Length of B – A
        bool HasCollision = false;
    };

    struct Collision final
    {
        std::weak_ptr<CollisionBody> BodyA;
        std::weak_ptr<CollisionBody> BodyB;
        CollisionPoints Points;

        bool operator==(const Collision& other) const
        {
            return this->BodyA.lock() == other.BodyA.lock()
                && this->BodyB.lock() == other.BodyB.lock() && this->Points.A == other.Points.A
                && this->Points.B == other.Points.B && this->Points.Normal == other.Points.Normal
                && this->Points.Depth == other.Points.Depth
                && this->Points.HasCollision == other.Points.HasCollision;
        }
    };

    class CollisionBody
    {
    public:
        using CollisionCallback = std::function<void(Collision)>;

        friend class CollisionWorld;

    public:
        CollisionBody() noexcept = default;

        explicit CollisionBody(const Vect& position) { SetPosition(position); }

        explicit CollisionBody(const std::shared_ptr<Collider>& collider)
            : _collider(collider)
        {}

        CollisionBody(const Vect& position, const std::shared_ptr<Collider>& collider)
            : _collider(collider)
        {
            SetPosition(position);
        }

        CollisionBody(const CollisionBody&) = default;
        CollisionBody(CollisionBody&&) = default;
        CollisionBody& operator=(const CollisionBody&) = default;
        CollisionBody& operator=(CollisionBody&&) = default;
        virtual ~CollisionBody() = default;

        [[nodiscard]] constexpr const Vect& GetPosition() const noexcept { return _position; }

        constexpr void SetPosition(const Vect& position) noexcept
        {
            if (!IsFinite(position))
            {
                _position = {0.0F, 0.0F};
                return;
            }

            _position = position;
        }

        constexpr void UpdatePosition(const Vect& delta) noexcept
        {
            if (!IsFinite(delta)) return;

            _position += delta;
        }

        [[nodiscard]] constexpr const std::shared_ptr<Collider>& GetCollider() const noexcept
        {
            return _collider;
        }

        constexpr void SetCollider(const std::shared_ptr<Collider>& collider) noexcept
        {
            _collider = collider;
        }

        constexpr void RemoveCollider() noexcept { _collider.reset(); }

        [[nodiscard]] constexpr Vect GetColliderCenterWorld() const noexcept
        {
            if (!_collider) return _position;

            return _position + _collider->GetCenterLocal();
        }

        void BindOnBeginOverlap(CollisionCallback callback) noexcept
        {
            _onBeginOverlap = std::move(callback);
        }

        void BindOnEndOverlap(CollisionCallback callback) noexcept
        {
            _onEndOverlap = std::move(callback);
        }

    protected:
        constexpr void InvokeOnBeginOverlap(const Collision& collision) const
        {
            if (_onBeginOverlap)
            {
                _onBeginOverlap(collision);
            }
        }

        constexpr void InvokeOnEndOverlap(const Collision& collision) const
        {
            if (_onEndOverlap)
            {
                _onEndOverlap(collision);
            }
        }

    protected:
        // Position of the object in the 2D space, in meters (m)
        Vect _position = {0.0F, 0.0F};

        // Collider associated with this collision body
        std::shared_ptr<Collider> _collider;

        CollisionCallback _onBeginOverlap;
        CollisionCallback _onEndOverlap;
    };
}   // namespace Guch2D
