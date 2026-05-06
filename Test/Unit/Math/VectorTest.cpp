#include "Math/Vector.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(VectorTest, IsFiniteTrue)
    {
        constexpr Guch2D::Vect finiteVec(1.0F, -2.0F);
        EXPECT_TRUE(Guch2D::IsFinite(finiteVec));
    }

    TEST(VectorTest, IsFiniteFalseNaN)
    {
        constexpr Guch2D::Vect nanVec(NAN, 2.0F);
        EXPECT_FALSE(Guch2D::IsFinite(nanVec));
    }

    TEST(VectorTest, IsFiniteFalseInfinity)
    {
        constexpr Guch2D::Vect infVec(INFINITY, -3.0F);
        EXPECT_FALSE(Guch2D::IsFinite(infVec));
    }

    TEST(VectorTest, VectLengthPositive)
    {
        constexpr Guch2D::Vect vec(3.0F, 4.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectLength(vec), 5.0F);
    }

    TEST(VectorTest, VectLenghtNegative)
    {
        constexpr Guch2D::Vect vec(-3.0F, -4.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectLength(vec), 5.0F);
    }

    TEST(VectorTest, VectLenghtZero)
    {
        constexpr Guch2D::Vect vec(0.0F, 0.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectLength(vec), 0.0F);
    }

    TEST(VectorTest, VectLenghtNaN)
    {
        constexpr Guch2D::Vect vec(NAN, 4.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectLength(vec), 0.0F);
    }

    TEST(VectorTest, VectLenghtInfinity)
    {
        constexpr Guch2D::Vect vec(INFINITY, 4.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectLength(vec), 0.0F);
    }

    TEST(VectorTest, VectNormalizePositive)
    {
        constexpr Guch2D::Vect vec(3.0F, 4.0F);
        const Guch2D::Vect normalized = Guch2D::VectNormalize(vec);
        EXPECT_FLOAT_EQ(normalized.x, 0.6F);
        EXPECT_FLOAT_EQ(normalized.y, 0.8F);
    }

    TEST(VectorTest, VectNormalizeNegative)
    {
        constexpr Guch2D::Vect vec(-3.0F, -4.0F);
        const Guch2D::Vect normalized = Guch2D::VectNormalize(vec);
        EXPECT_FLOAT_EQ(normalized.x, -0.6F);
        EXPECT_FLOAT_EQ(normalized.y, -0.8F);
    }

    TEST(VectorTest, VectNormalizeZero)
    {
        constexpr Guch2D::Vect vec(0.0F, 0.0F);
        const Guch2D::Vect normalized = Guch2D::VectNormalize(vec);
        EXPECT_FLOAT_EQ(normalized.x, 0.0F);
        EXPECT_FLOAT_EQ(normalized.y, 0.0F);
    }

    TEST(VectorTest, VectNormalizeNaN)
    {
        constexpr Guch2D::Vect vec(NAN, 4.0F);
        const Guch2D::Vect normalized = Guch2D::VectNormalize(vec);
        EXPECT_FLOAT_EQ(normalized.x, 0.0F);
        EXPECT_FLOAT_EQ(normalized.y, 0.0F);
    }

    TEST(VectorTest, VectNormalizeInfinity)
    {
        constexpr Guch2D::Vect vec(INFINITY, 4.0F);
        const Guch2D::Vect normalized = Guch2D::VectNormalize(vec);
        EXPECT_FLOAT_EQ(normalized.x, 0.0F);
        EXPECT_FLOAT_EQ(normalized.y, 0.0F);
    }

    TEST(VectorTest, VectDotPositive)
    {
        constexpr Guch2D::Vect vect1(1.0F, 2.0F);
        constexpr Guch2D::Vect vect2(3.0F, 4.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectDot(vect1, vect2), 11.0F);
    }

    TEST(VectorTest, VectDotOrthogonal)
    {
        constexpr Guch2D::Vect vect1(1.0F, 0.0F);
        constexpr Guch2D::Vect vect2(0.0F, 5.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectDot(vect1, vect2), 0.0F);
    }

    TEST(VectorTest, VectDotOppositeDirection)
    {
        constexpr Guch2D::Vect vect1(2.0F, -3.0F);
        constexpr Guch2D::Vect vect2(-4.0F, 6.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectDot(vect1, vect2), -26.0F);
    }

    TEST(VectorTest, VectDotWithZeroVector)
    {
        constexpr Guch2D::Vect vect1(0.0F, 0.0F);
        constexpr Guch2D::Vect vect2(4.0F, -7.0F);
        EXPECT_FLOAT_EQ(Guch2D::VectDot(vect1, vect2), 0.0F);
    }
}   // namespace
