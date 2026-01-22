#include "Collision/Collider.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(ColliderTest, DefaultConstructor)
    {
        const Guch2D::Collider collider;
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::None);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(ColliderTest, CenterConstructor)
    {
        constexpr Guch2D::Vect center {1.0F, 2.0F};
        const Guch2D::Collider collider(center);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::None);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(CollierTest, SetColliderType)
    {
        Guch2D::Collider collider;
        collider.SetColliderType(Guch2D::ColliderType::Circle);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::Circle);
    }

    TEST(ColliderTest, SetCenterLocalPositive)
    {
        Guch2D::Collider collider;
        constexpr Guch2D::Vect center {3.0F, 4.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(ColliderTest, SetCenterLocalNegative)
    {
        Guch2D::Collider collider;
        constexpr Guch2D::Vect center {-5.0F, -6.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(ColliderTest, SetCenterLocalZero)
    {
        Guch2D::Collider collider;
        constexpr Guch2D::Vect center {0.0F, 0.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(ColliderTest, SetCenterLocalNaN)
    {
        Guch2D::Collider collider;
        constexpr Guch2D::Vect center {NAN, 0.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(ColliderTest, SetCenterLocalInfinite)
    {
        Guch2D::Collider collider;
        constexpr Guch2D::Vect center {INFINITY, 0.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
    }

}   // namespace
