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
            {"Zero",                     0.0F,                    0.0F      },
            {"NegativeZero",             -0.0F,                   0.0F      },
            {"PositiveHalfPi",           Pi * 0.5F,               Pi * 0.5F },
            {"PositivePi",               Pi,                      Pi        },
            {"PositiveBeforeFullTurn",   TwoPi - 0.25F,           -0.25F    },
            {"PositiveFullTurn",         TwoPi,                   0.0F      },
            {"PositiveMoreThanFullTurn", TwoPi + (Pi * 0.5F),     Pi * 0.5F },
            {"PositiveMultipleTurns",    (TwoPi * 5.0F) + 1.25F,  1.25F     },
            {"NegativeHalfPi",           Pi * -0.5F,              Pi * -0.5F},
            {"NegativePi",               -Pi,                     Pi        },
            {"NegativeFullTurn",         -TwoPi,                  0.0F      },
            {"NegativeMoreThanFullTurn", -TwoPi - (Pi * 0.5F),    Pi * -0.5F},
            {"NegativeMultipleTurns",    (TwoPi * -5.0F) - 1.25F, -1.25F    }
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
            EXPECT_GT(normalized, -Pi);
            EXPECT_LE(normalized, Pi);
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

    TEST(RotatorTest, NormalizeRotatorDegreesNormalizesFiniteInputs)
    {
        const NormalizeRotatorCase cases[] = {
            {"Zero",                     0.0F,             0.0F  },
            {"NegativeZero",             -0.0F,            0.0F  },
            {"PositiveNinety",           90.0F,            90.0F },
            {"PositiveOneHundredEighty", 180.0F,           180.0F},
            {"PositiveBeforeFullTurn",   359.0F,           -1.0F },
            {"PositiveFullTurn",         360.0F,           0.0F  },
            {"PositiveMoreThanFullTurn", 450.0F,           90.0F },
            {"PositiveMultipleTurns",    1080.0F + 45.0F,  45.0F },
            {"NegativeNinety",           -90.0F,           -90.0F},
            {"NegativeOneHundredEighty", -180.0F,          180.0F},
            {"NegativeFullTurn",         -360.0F,          0.0F  },
            {"NegativeMoreThanFullTurn", -450.0F,          -90.0F},
            {"NegativeMultipleTurns",    -1080.0F - 45.0F, -45.0F}
        };

        for (const NormalizeRotatorCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            EXPECT_NEAR(Guch2D::NormalizeRotatorDegrees(testCase.Input),
                        testCase.Expected,
                        RotationTolerance);
        }
    }

    TEST(RotatorTest, NormalizeRotatorDegreesKeepsFiniteResultsInCanonicalRange)
    {
        const Guch2D::Rotator inputs[] = {0.0F,
                                          -0.0F,
                                          180.0F,
                                          -180.0F,
                                          3600.0F + 60.0F,
                                          -3600.0F - 60.0F,
                                          std::numeric_limits<float>::min(),
                                          -std::numeric_limits<float>::min(),
                                          std::numeric_limits<float>::max(),
                                          std::numeric_limits<float>::lowest()};

        for (const Guch2D::Rotator input : inputs)
        {
            SCOPED_TRACE(input);
            const Guch2D::Rotator normalized = Guch2D::NormalizeRotatorDegrees(input);

            EXPECT_TRUE(std::isfinite(normalized));
            EXPECT_GT(normalized, -180.0F);
            EXPECT_LE(normalized, 180.0F);
        }
    }

    TEST(RotatorTest, NormalizeRotatorDegreesInvalidInputsReturnNaN)
    {
        const Guch2D::Rotator inputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator input : inputs)
        {
            SCOPED_TRACE(input);
            EXPECT_TRUE(std::isnan(Guch2D::NormalizeRotatorDegrees(input)));
        }
    }
}   // namespace
