#include "Collision/CollisionBody.hpp"

#include <gtest/gtest.h>

namespace
{
    // Test helper that exposes protected invoke methods as public for testing.
    class TestableCollisionBody final : public Guch2D::CollisionBody
    {
    public:
        using Guch2D::CollisionBody::CollisionCallback;
        using Guch2D::CollisionBody::InvokeOnBeginOverlap;
        using Guch2D::CollisionBody::InvokeOnEndOverlap;
    };

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

    TEST(CollisionBodyTest, SetPositionNegative)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {-7.0F, -8.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollisionBodyTest, SetPositionZero)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {0.0F, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollisionBodyTest, SetPositionNaN)
    {
        Guch2D::CollisionBody body;
        constexpr Guch2D::Vect newPosition {NAN, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, SetPositionInfinite)
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

    TEST(CollisionBodyTest, SetColliderNullptr)
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

    TEST(CollisionBodyTest, GetColliderCenterWorldWithCollider)
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

    TEST(CollisionBodyTest, InvokeOnBeginOverlap)
    {
        TestableCollisionBody body;
        bool callbackInvoked = false;
        body.BindOnBeginOverlap(
            [&callbackInvoked](const Guch2D::Collision& /*collision*/) { callbackInvoked = true; });

        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        EXPECT_TRUE(callbackInvoked);
    }

    TEST(CollisionBodyTest, InvokeOnEndOverlap)
    {
        TestableCollisionBody body;
        bool callbackInvoked = false;
        body.BindOnEndOverlap(
            [&callbackInvoked](const Guch2D::Collision& /*collision*/) { callbackInvoked = true; });

        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        EXPECT_TRUE(callbackInvoked);
    }

    TEST(CollisionBodyTest, InvokeOnBeginOverlapNoCallback)
    {
        const TestableCollisionBody body;
        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        SUCCEED();
    }

    TEST(CollisionBodyTest, InvokeOnEndOverlapNoCallback)
    {
        const TestableCollisionBody body;
        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        SUCCEED();
    }

    TEST(CollisionBodyTest, BindOnBeginOverlapReplacesCallback)
    {
        TestableCollisionBody body;
        bool firstCallbackInvoked = false;
        bool secondCallbackInvoked = false;

        body.BindOnBeginOverlap([&firstCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            firstCallbackInvoked = true;
        });
        body.BindOnBeginOverlap([&secondCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            secondCallbackInvoked = true;
        });

        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        EXPECT_FALSE(firstCallbackInvoked);
        EXPECT_TRUE(secondCallbackInvoked);
    }

    TEST(CollsionBodyTest, BindOnEndOverlapReplacesCallback)
    {
        TestableCollisionBody body;
        bool firstCallbackInvoked = false;
        bool secondCallbackInvoked = false;

        body.BindOnEndOverlap([&firstCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            firstCallbackInvoked = true;
        });
        body.BindOnEndOverlap([&secondCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            secondCallbackInvoked = true;
        });

        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        EXPECT_FALSE(firstCallbackInvoked);
        EXPECT_TRUE(secondCallbackInvoked);
    }

    TEST(CollisionBodyTest, BindOnBeginOVverlapNullptr)
    {
        TestableCollisionBody body;
        body.BindOnBeginOverlap(nullptr);

        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        SUCCEED();
    }

    TEST(CollisionBodyTest, BindOnEndOverlapNullptr)
    {
        TestableCollisionBody body;
        body.BindOnEndOverlap(nullptr);

        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        SUCCEED();
    }

}   // namespace
