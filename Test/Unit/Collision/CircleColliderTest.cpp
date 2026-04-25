#include "Collision/CircleCollider.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace
{
    TEST(CircleColliderTest, DefaultConstructor)
    {
        const Guch2D::CircleCollider collider;
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::Circle);
        EXPECT_FLOAT_EQ(collider.GetRadius(), 0.0F);
    }

    TEST(CircelColliderTest, RadiusConstructor)
    {
        constexpr float radius = 5.0F;
        const Guch2D::CircleCollider collider(radius);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::Circle);
        EXPECT_FLOAT_EQ(collider.GetRadius(), radius);
    }

    TEST(CirecleColliderTest, SetRadiusPositive)
    {
        Guch2D::CircleCollider collider;
        constexpr float radius = 10.0F;
        collider.SetRadius(radius);
        EXPECT_FLOAT_EQ(collider.GetRadius(), radius);
    }

    TEST(CircleColliderTest, SetRadiusNegative)
    {
        Guch2D::CircleCollider collider;
        constexpr float radius = -3.0F;
        collider.SetRadius(radius);
        EXPECT_FLOAT_EQ(collider.GetRadius(), 0.0F);
    }

    TEST(CircleColliderTest, SetRadiusZero)
    {
        Guch2D::CircleCollider collider;
        constexpr float radius = 0.0F;
        collider.SetRadius(radius);
        EXPECT_FLOAT_EQ(collider.GetRadius(), radius);
    }

    TEST(CircleColliderTest, SetRadiusInfinite)
    {
        Guch2D::CircleCollider collider;
        constexpr float radius = INFINITY;
        collider.SetRadius(radius);
        EXPECT_FLOAT_EQ(collider.GetRadius(), 0.0F);
    }

    TEST(CircleColliderTest, SetRadiusNaN)
    {
        Guch2D::CircleCollider collider;
        constexpr float radius = NAN;
        collider.SetRadius(radius);
        EXPECT_FLOAT_EQ(collider.GetRadius(), 0.0F);
    }

    TEST(CircleColliderTest, CenterRadiusConstructor)
    {
        constexpr Guch2D::Vect center {2.0F, 3.0F};
        constexpr float radius = 4.0F;
        const Guch2D::CircleCollider collider(center, radius);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::Circle);
        EXPECT_EQ(collider.GetCenterLocal(), center);
        EXPECT_FLOAT_EQ(collider.GetRadius(), radius);
    }

    TEST(CircleColliderTest, BordersWithZeroCenter)
    {
        const Guch2D::CircleCollider collider(3.0F);

        EXPECT_EQ(collider.LeftBorder(), Guch2D::Vect(-3.0F, 0.0F));
        EXPECT_EQ(collider.RightBorder(), Guch2D::Vect(3.0F, 0.0F));
        EXPECT_EQ(collider.TopBorder(), Guch2D::Vect(0.0F, 3.0F));
        EXPECT_EQ(collider.BottomBorder(), Guch2D::Vect(0.0F, -3.0F));
    }

    TEST(CircleColliderTest, BordersWithCustomCenter)
    {
        const Guch2D::CircleCollider collider({2.0F, -1.0F}, 4.0F);

        EXPECT_EQ(collider.LeftBorder(), Guch2D::Vect(-2.0F, -1.0F));
        EXPECT_EQ(collider.RightBorder(), Guch2D::Vect(6.0F, -1.0F));
        EXPECT_EQ(collider.TopBorder(), Guch2D::Vect(2.0F, 3.0F));
        EXPECT_EQ(collider.BottomBorder(), Guch2D::Vect(2.0F, -5.0F));
    }

    TEST(CircleColliderTest, BordersWithZeroRadiusMatchCenter)
    {
        const Guch2D::CircleCollider collider({-9.0F, 5.0F}, 0.0F);

        EXPECT_EQ(collider.LeftBorder(), collider.GetCenterLocal());
        EXPECT_EQ(collider.RightBorder(), collider.GetCenterLocal());
        EXPECT_EQ(collider.TopBorder(), collider.GetCenterLocal());
        EXPECT_EQ(collider.BottomBorder(), collider.GetCenterLocal());
    }

    TEST(CircleColliderTest, GetAreaMatchesKnownValues)
    {
        struct TestCase
        {
            float Radius;
            float ExpectedArea;
        };

        const TestCase testCases[] = {
            {0.0F, 0.0F},
            {0.5F, 0.78539816F},
            {2.0F, 12.56637061F},
            {5.0F, 78.53981634F}
        };

        for (const auto& testCase : testCases)
        {
            SCOPED_TRACE(::testing::Message() << "radius=" << testCase.Radius);

            const Guch2D::CircleCollider collider(testCase.Radius);
            EXPECT_NEAR(collider.GetArea(), testCase.ExpectedArea, 1.0e-5F);
        }
    }

    TEST(CircleColliderTest, GetAreaScalesQuadraticallyWithRadius)
    {
        constexpr float baseRadius = 1.75F;
        constexpr float scaledRadius = baseRadius * 2.0F;

        const Guch2D::CircleCollider baseCollider(baseRadius);
        const Guch2D::CircleCollider scaledCollider(scaledRadius);

        EXPECT_NEAR(scaledCollider.GetArea(), baseCollider.GetArea() * 4.0F, 1.0e-5F);
    }

    TEST(CircleColliderTest, GetAreaIsIndependentFromCenter)
    {
        constexpr float radius = 4.0F;
        const Guch2D::CircleCollider colliderA({0.0F, 0.0F}, radius);
        const Guch2D::CircleCollider colliderB({123.0F, -456.0F}, radius);

        EXPECT_FLOAT_EQ(colliderA.GetArea(), colliderB.GetArea());
    }

    TEST(CircleColliderTest, GetAreaIsZeroForInvalidRadiusInput)
    {
        const Guch2D::CircleCollider negativeRadiusCollider(-3.0F);
        const Guch2D::CircleCollider infiniteRadiusCollider(INFINITY);
        const Guch2D::CircleCollider nanRadiusCollider(NAN);

        EXPECT_FLOAT_EQ(negativeRadiusCollider.GetArea(), 0.0F);
        EXPECT_FLOAT_EQ(infiniteRadiusCollider.GetArea(), 0.0F);
        EXPECT_FLOAT_EQ(nanRadiusCollider.GetArea(), 0.0F);
    }
}   // namespace
