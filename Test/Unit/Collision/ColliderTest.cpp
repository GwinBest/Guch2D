#include "Collision/Collider.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestCollider final : public Guch2D::Collider
    {
    public:
        using Guch2D::Collider::Collider;

        [[nodiscard]] Guch2D::Vect LeftBorder() const noexcept override { return GetCenterLocal(); }

        [[nodiscard]] Guch2D::Vect RightBorder() const noexcept override
        {
            return GetCenterLocal();
        }

        [[nodiscard]] Guch2D::Vect TopBorder() const noexcept override { return GetCenterLocal(); }

        [[nodiscard]] Guch2D::Vect BottomBorder() const noexcept override
        {
            return GetCenterLocal();
        }
    };
    
    TEST(ColliderTest, DefaultConstructor)
    {
        const TestCollider collider;
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::None);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(ColliderTest, CenterConstructor)
    {
        constexpr Guch2D::Vect center {1.0F, 2.0F};
        const TestCollider collider(center);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::None);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(CollierTest, SetColliderType)
    {
        TestCollider collider;
        collider.SetColliderType(Guch2D::ColliderType::Circle);
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::Circle);
    }

    TEST(ColliderTest, SetCenterLocalPositive)
    {
        TestCollider collider;
        constexpr Guch2D::Vect center {3.0F, 4.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(ColliderTest, SetCenterLocalNegative)
    {
        TestCollider collider;
        constexpr Guch2D::Vect center {-5.0F, -6.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(ColliderTest, SetCenterLocalZero)
    {
        TestCollider collider;
        constexpr Guch2D::Vect center {0.0F, 0.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), center);
    }

    TEST(ColliderTest, SetCenterLocalNaN)
    {
        TestCollider collider;
        constexpr Guch2D::Vect center {NAN, 0.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(ColliderTest, SetCenterLocalInfinite)
    {
        TestCollider collider;
        constexpr Guch2D::Vect center {INFINITY, 0.0F};
        collider.SetCenterLocal(center);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
    }

}   // namespace
