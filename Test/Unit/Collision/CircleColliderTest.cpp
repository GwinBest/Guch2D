#include "Collision/CircleCollider.hpp"

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
}   // namespace
