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
        constexpr Guch2D::Vect center{2.0F, 3.0F};
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
}   // namespace
