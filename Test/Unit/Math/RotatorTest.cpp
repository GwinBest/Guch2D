#include "Math/Rotator.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <numbers>

namespace
{
    constexpr float Pi = std::numbers::pi_v<float>;
    constexpr float TwoPi = Pi * 2.0F;
    constexpr float RotationTolerance = 1.0e-4F;

    struct NormalizeRotatorCase
    {
        const char* Name;
        Guch2D::Rotator Input;
        Guch2D::Rotator Expected;
    };

    TEST(RotatorTest, NormalizeRotatorNormalizesFiniteInputs)
    {
        const NormalizeRotatorCase cases[] = {
            {"Zero",                     0.0F,                    0.0F         },
            {"NegativeZero",             -0.0F,                   0.0F         },
            {"PositiveHalfPi",           Pi * 0.5F,               Pi * 0.5F    },
            {"PositivePi",               Pi,                      Pi           },
            {"PositiveBeforeFullTurn",   TwoPi - 0.25F,           TwoPi - 0.25F},
            {"PositiveFullTurn",         TwoPi,                   0.0F         },
            {"PositiveMoreThanFullTurn", TwoPi + (Pi * 0.5F),     Pi * 0.5F    },
            {"PositiveMultipleTurns",    (TwoPi * 5.0F) + 1.25F,  1.25F        },
            {"NegativeHalfPi",           Pi * -0.5F,              Pi * 1.5F    },
            {"NegativePi",               -Pi,                     Pi           },
            {"NegativeFullTurn",         -TwoPi,                  0.0F         },
            {"NegativeMoreThanFullTurn", -TwoPi - (Pi * 0.5F),    Pi * 1.5F    },
            {"NegativeMultipleTurns",    (TwoPi * -5.0F) - 1.25F, TwoPi - 1.25F}
        };

        for (const NormalizeRotatorCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            EXPECT_NEAR(Guch2D::NormalizeRotator(testCase.Input),
                        testCase.Expected,
                        RotationTolerance);
        }
    }

    TEST(RotatorTest, NormalizeRotatorKeepsFiniteResultsInCanonicalRange)
    {
        const Guch2D::Rotator inputs[] = {0.0F,
                                          -0.0F,
                                          Pi,
                                          -Pi,
                                          (TwoPi * 10.0F) + (Pi / 3.0F),
                                          (TwoPi * -10.0F) - (Pi / 3.0F),
                                          std::numeric_limits<float>::min(),
                                          -std::numeric_limits<float>::min(),
                                          std::numeric_limits<float>::max(),
                                          std::numeric_limits<float>::lowest()};

        for (const Guch2D::Rotator input : inputs)
        {
            SCOPED_TRACE(input);
            const Guch2D::Rotator normalized = Guch2D::NormalizeRotator(input);

            EXPECT_TRUE(std::isfinite(normalized));
            EXPECT_GE(normalized, 0.0F);
            EXPECT_LT(normalized, TwoPi);
        }
    }

    TEST(RotatorTest, NormalizeRotatorInvalidInputsReturnNaN)
    {
        const Guch2D::Rotator inputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator input : inputs)
        {
            SCOPED_TRACE(input);
            EXPECT_TRUE(std::isnan(Guch2D::NormalizeRotator(input)));
        }
    }
}   // namespace
