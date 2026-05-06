#pragma once

#include <array>
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
        std::array<Vect, 2> ContactPoints = {};
        Vect Normal = {0.0F, 0.0F};   // ContactPoints.back() - ContactPoints.front() normalized
        float Depth = 0.0F;           // Length of ContactPoints.back() - ContactPoints.front()
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
                && this->BodyB.lock() == other.BodyB.lock()
                && this->Points.ContactPoints == other.Points.ContactPoints
                && this->Points.Normal == other.Points.Normal
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
        virtual ~CollisionBody() = 0;

        [[nodiscard]] const Vect& GetPosition() const noexcept { return _position; }

        void SetPosition(const Vect& position) noexcept
        {
            if (!IsFinite(position))
            {
                _position = {0.0F, 0.0F};
                return;
            }

            _position = position;
        }

        void UpdatePosition(const Vect& delta) noexcept
        {
            if (!IsFinite(delta))
                return;

            _position += delta;
        }

        [[nodiscard]] const std::shared_ptr<Collider>& GetCollider() const noexcept
        {
            return _collider;
        }

        void SetCollider(const std::shared_ptr<Collider>& collider) noexcept
        {
            _collider = collider;
        }

        void RemoveCollider() noexcept { _collider.reset(); }

        [[nodiscard]] Vect GetColliderCenterWorld() const noexcept
        {
            if (!_collider)
                return {};

            return _position + _collider->GetCenterLocal();
        }

        [[nodiscard]] Vect GetColliderLeftBorderWorld() const noexcept
        {
            if (!_collider)
                return {};

            return _position + _collider->LeftBorder();
        }

        [[nodiscard]] Vect GetColliderRightBorderWorld() const noexcept
        {
            if (!_collider)
                return {};

            return _position + _collider->RightBorder();
        }

        [[nodiscard]] Vect GetColliderTopBorderWorld() const noexcept
        {
            if (!_collider)
                return {};

            return _position + _collider->TopBorder();
        }

        [[nodiscard]] Vect GetColliderBottomBorderWorld() const noexcept
        {
            if (!_collider)
                return {};

            return _position + _collider->BottomBorder();
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
        void InvokeOnBeginOverlap(const Collision& collision) const
        {
            if (_onBeginOverlap)
            {
                _onBeginOverlap(collision);
            }
        }

        void InvokeOnEndOverlap(const Collision& collision) const
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

    inline CollisionBody::~CollisionBody() = default;
}   // namespace Guch2D
