#include "Collision/AABBCollider.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace
{
    TEST(AABBColliderTest, DefaultConstructor)
    {
        const Guch2D::AABBCollider collider;
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::AABB);
        EXPECT_EQ(collider.GetExtent(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(AABBColliderTest, ExtentConstructor)
    {
        constexpr Guch2D::Vect extent{3.0F, 4.0F};
        const Guch2D::AABBCollider collider(extent);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::AABB);
        EXPECT_EQ(collider.GetExtent(), extent);
    }

    TEST(AABBColliderTest, CenterExtentConstructor)
    {
        constexpr Guch2D::Vect center{1.0F, 2.0F};
        constexpr Guch2D::Vect extent{5.0F, 6.0F};
        const Guch2D::AABBCollider collider(center, extent);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::AABB);
        EXPECT_EQ(collider.GetCenterLocal(), center);
        EXPECT_EQ(collider.GetExtent(), extent);
    }

    TEST(AABBColliderTest, SetExtentPositive)
    {
        Guch2D::AABBCollider collider;
        constexpr Guch2D::Vect extent{7.0F, 8.0F};
        collider.SetExtent(extent);
        EXPECT_EQ(collider.GetExtent(), extent);
    }

    TEST(AABBColliderTest, SetExtentZero)
    {
        Guch2D::AABBCollider collider;
        constexpr Guch2D::Vect extent{0.0F, 0.0F};
        collider.SetExtent(extent);
        EXPECT_EQ(collider.GetExtent(), extent);
    }

    TEST(AABBColliderTest, SetExtentNaN)
    {
        Guch2D::AABBCollider collider;
        constexpr Guch2D::Vect extent{NAN, 0.0F};
        collider.SetExtent(extent);
        EXPECT_EQ(collider.GetExtent(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(AABBColliderTest, SetExtentInfinite)
    {
        Guch2D::AABBCollider collider;
        constexpr Guch2D::Vect extent{INFINITY, 0.0F};
        collider.SetExtent(extent);
        EXPECT_EQ(collider.GetExtent(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(AABBColliderTest, BordersWithZeroCenter)
    {
        const Guch2D::AABBCollider collider({2.0F, 3.0F});

        EXPECT_EQ(collider.LeftBorder(), Guch2D::Vect(-2.0F, 0.0F));
        EXPECT_EQ(collider.RightBorder(), Guch2D::Vect(2.0F, 0.0F));
        EXPECT_EQ(collider.TopBorder(), Guch2D::Vect(0.0F, 3.0F));
        EXPECT_EQ(collider.BottomBorder(), Guch2D::Vect(0.0F, -3.0F));
    }

    TEST(AABBColliderTest, BordersWithCustomCenter)
    {
        const Guch2D::AABBCollider collider({1.0F, -2.0F}, {3.0F, 4.0F});

        EXPECT_EQ(collider.LeftBorder(), Guch2D::Vect(-2.0F, -2.0F));
        EXPECT_EQ(collider.RightBorder(), Guch2D::Vect(4.0F, -2.0F));
        EXPECT_EQ(collider.TopBorder(), Guch2D::Vect(1.0F, 2.0F));
        EXPECT_EQ(collider.BottomBorder(), Guch2D::Vect(1.0F, -6.0F));
    }

    TEST(AABBColliderTest, BordersWithZeroExtentMatchCenter)
    {
        const Guch2D::AABBCollider collider({-7.0F, 11.0F}, {0.0F, 0.0F});

        EXPECT_EQ(collider.LeftBorder(), collider.GetCenterLocal());
        EXPECT_EQ(collider.RightBorder(), collider.GetCenterLocal());
        EXPECT_EQ(collider.TopBorder(), collider.GetCenterLocal());
        EXPECT_EQ(collider.BottomBorder(), collider.GetCenterLocal());
    }
}
