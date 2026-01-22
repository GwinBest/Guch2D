#include "Collision/CollisionBody.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(CollisionBodyTest, DefaultConstructor)
    {
        const Guch2D::CollisionBody body;
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position {1.0F, 2.0F};
        const Guch2D::CollisionBody body(position);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, ColliderConstructor)
    {
        const auto collider = std::make_shared<Guch2D::Collider>();
        const Guch2D::CollisionBody body(collider);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), collider);
    }

    TEST(CollisionBodyTest, PositionAndColliderConstructor)
    {
        constexpr Guch2D::Vect position {3.0F, 4.0F};
        const auto collider = std::make_shared<Guch2D::Collider>();
        const Guch2D::CollisionBody body(position, collider);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), collider);
    }

    TEST(CollisionBodyTest, SetPositionPositive)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {5.0F, 6.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollsionBodyTest, SetPositionNegative)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {-7.0F, -8.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollsionBodyTest, SetPositionZero)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {0.0F, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollsionBodyTest, SetPositionNaN)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {NAN, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollsionBodyTest, SetPositionInfinite)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {INFINITY, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, UpdatePositionPositive)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect delta {2.0F, 3.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), delta);
    }

    TEST(CollisionBodyTest, UpdatePositionNegative)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect delta {-4.0F, -5.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), delta);
    }

    TEST(CollisionBodyTest, UpdatePositionZero)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect delta {0.0F, 0.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), delta);
    }

    TEST(CollisionBodyTest, UpdatePositionNaN)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect delta {NAN, 0.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, UpdatePositionInfinite)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect delta {INFINITY, 0.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, SetCollider)
    {
        Guch2D::CollisionBody body;
        const auto collider = std::make_shared<Guch2D::Collider>();
        body.SetCollider(collider);
        EXPECT_EQ(body.GetCollider(), collider);
    }

    TEST(CollsioinBodyTest, SetColliderNullptr)
    {
        Guch2D::CollisionBody body;
        const std::shared_ptr<Guch2D::Collider> collider = nullptr;
        body.SetCollider(collider);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, RemoveCollider)
    {
        Guch2D::CollisionBody body;
        const auto collider = std::make_shared<Guch2D::Collider>();
        body.SetCollider(collider);
        body.RemoveCollider();
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, GetColliderCenterWorldNoCollider)
    {
        const Guch2D::CollisionBody body;
        EXPECT_EQ(body.GetColliderCenterWorld(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollsionBodyTest, GetColliderCenterWorldWithCollider)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect position {1.0F, 2.0F};
        body.SetPosition(position);
        const auto collider = std::make_shared<Guch2D::Collider>();
        constexpr Guch2D::Vect colliderCenter {3.0F, 4.0F};
        collider->SetCenterLocal(colliderCenter);
        body.SetCollider(collider);
        EXPECT_EQ(body.GetColliderCenterWorld(), position + colliderCenter);
    }

}   // namespace
