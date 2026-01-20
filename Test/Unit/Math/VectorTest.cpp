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
}   // namespace
