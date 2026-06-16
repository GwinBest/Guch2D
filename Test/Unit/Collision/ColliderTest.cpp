#include "Collision/Collider.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    constexpr float Pi = std::numbers::pi_v<float>;
    constexpr float DegToRad = Pi / 180.0F;
    constexpr float RotationTolerance = 1.0e-4F;
    constexpr float DegreeTolerance = 1.0e-3F;

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

        [[nodiscard]] float GetArea() const noexcept override { return 0.0f; }
    };

    TEST(ColliderTest, DefaultConstructor)
    {
        const TestCollider collider;
        EXPECT_EQ(collider.GetColliderType(), Guch2D::ColliderType::None);
        EXPECT_EQ(collider.GetCenterLocal(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_FLOAT_EQ(collider.GetRotationLocal(), 0.0F);
        EXPECT_FLOAT_EQ(collider.GetRotationDegreesLocal(), 0.0F);
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

    TEST(ColliderTest, SetRotationLocalStoresFiniteRadians)
    {
        TestCollider collider;

        collider.SetRotationLocal(Pi * 0.5F);

        EXPECT_NEAR(collider.GetRotationLocal(), Pi * 0.5F, RotationTolerance);
        EXPECT_NEAR(collider.GetRotationDegreesLocal(), 90.0F, DegreeTolerance);
    }

    TEST(ColliderTest, SetRotationLocalResetsInvalidRadiansToZero)
    {
        const Guch2D::Rotator invalidInputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator invalidInput : invalidInputs)
        {
            SCOPED_TRACE(invalidInput);
            TestCollider collider;
            collider.SetRotationLocal(Pi * 0.5F);

            collider.SetRotationLocal(invalidInput);

            EXPECT_FLOAT_EQ(collider.GetRotationLocal(), 0.0F);
            EXPECT_FLOAT_EQ(collider.GetRotationDegreesLocal(), 0.0F);
        }
    }

    TEST(ColliderTest, SetRotationDegreesLocalStoresFiniteDegrees)
    {
        TestCollider collider;

        collider.SetRotationDegreesLocal(90.0F);

        EXPECT_NEAR(collider.GetRotationLocal(), 90.0F * DegToRad, RotationTolerance);
        EXPECT_NEAR(collider.GetRotationDegreesLocal(), 90.0F, DegreeTolerance);
    }

    TEST(ColliderTest, RotateNormalizesAccumulatedRadians)
    {
        TestCollider collider;
        collider.SetRotationLocal(Pi * 0.25F);

        collider.Rotate(Pi);

        EXPECT_NEAR(collider.GetRotationLocal(), Pi * -0.75F, RotationTolerance);
        EXPECT_NEAR(collider.GetRotationDegreesLocal(), -135.0F, DegreeTolerance);
    }

    TEST(ColliderTest, RotateIgnoresInvalidRadians)
    {
        const Guch2D::Rotator invalidInputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator invalidInput : invalidInputs)
        {
            SCOPED_TRACE(invalidInput);
            TestCollider collider;
            collider.SetRotationLocal(Pi / 3.0F);

            collider.Rotate(invalidInput);

            EXPECT_NEAR(collider.GetRotationLocal(), Pi / 3.0F, RotationTolerance);
            EXPECT_NEAR(collider.GetRotationDegreesLocal(), 60.0F, DegreeTolerance);
        }
    }

    TEST(ColliderTest, RotateDegreesNormalizesAccumulatedDegrees)
    {
        TestCollider collider;
        collider.SetRotationDegreesLocal(30.0F);

        collider.RotateDegrees(-45.0F);

        EXPECT_NEAR(collider.GetRotationLocal(), -15.0F * DegToRad, RotationTolerance);
        EXPECT_NEAR(collider.GetRotationDegreesLocal(), -15.0F, DegreeTolerance);
    }

    TEST(ColliderTest, AABBIgnoresSetRotationLocal)
    {
        TestCollider collider;
        collider.SetColliderType(Guch2D::ColliderType::AABB);

        collider.SetRotationLocal(Pi * 0.5F);

        EXPECT_FLOAT_EQ(collider.GetRotationLocal(), 0.0F);
        EXPECT_FLOAT_EQ(collider.GetRotationDegreesLocal(), 0.0F);
    }

    TEST(ColliderTest, AABBIgnoresRotate)
    {
        TestCollider collider;
        collider.SetColliderType(Guch2D::ColliderType::AABB);

        collider.Rotate(Pi * 0.5F);
        collider.RotateDegrees(90.0F);

        EXPECT_FLOAT_EQ(collider.GetRotationLocal(), 0.0F);
        EXPECT_FLOAT_EQ(collider.GetRotationDegreesLocal(), 0.0F);
    }

}   // namespace
