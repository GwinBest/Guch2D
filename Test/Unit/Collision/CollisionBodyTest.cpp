#include "Collision/CollisionBody.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"

namespace
{
    // Test helper that exposes protected invoke methods as public for testing.
    class TestableCollisionBody final : public Guch2D::CollisionBody
    {
    public:
        using Guch2D::CollisionBody::CollisionBody;
        using Guch2D::CollisionBody::InvokeOnBeginOverlap;
        using Guch2D::CollisionBody::InvokeOnEndOverlap;
    };

    constexpr float Pi = std::numbers::pi_v<float>;
    constexpr float TwoPi = Pi * 2.0F;
    constexpr float DegToRad = Pi / 180.0F;
    constexpr float RotationTolerance = 1.0e-4F;
    constexpr float DegreeTolerance = 1.0e-3F;

    struct RotationCase
    {
        const char* Name;
        Guch2D::Rotator Input;
        Guch2D::Rotator Expected;
    };

    struct DegreeRotationCase
    {
        const char* Name;
        Guch2D::Rotator InputDegrees;
        Guch2D::Rotator ExpectedDegrees;
    };

    void ExpectRotationNear(const Guch2D::Rotator actual, const Guch2D::Rotator expected)
    {
        EXPECT_NEAR(actual, expected, RotationTolerance);
    }

    void ExpectDegreesNear(const Guch2D::Rotator actual, const Guch2D::Rotator expected)
    {
        EXPECT_NEAR(actual, expected, DegreeTolerance);
    }

    TEST(CollisionBodyTest, DefaultConstructor)
    {
        const TestableCollisionBody body;
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_FLOAT_EQ(body.GetRotation(), 0.0F);
        EXPECT_FLOAT_EQ(body.GetRotationDegrees(), 0.0F);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position {1.0F, 2.0F};
        const TestableCollisionBody body(position);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, ColliderConstructor)
    {
        const std::shared_ptr<Guch2D::Collider> collider = std::make_shared<Guch2D::CircleCollider>(
            1.0F);
        const TestableCollisionBody body(collider);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), collider);
    }

    TEST(CollisionBodyTest, PositionAndColliderConstructor)
    {
        constexpr Guch2D::Vect position {3.0F, 4.0F};
        const std::shared_ptr<Guch2D::Collider> collider = std::make_shared<Guch2D::CircleCollider>(
            1.0F);
        const TestableCollisionBody body(position, collider);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), collider);
    }

    TEST(CollisionBodyTest, SetPositionPositive)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect newPosition {5.0F, 6.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollisionBodyTest, SetPositionNegative)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect newPosition {-7.0F, -8.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollisionBodyTest, SetPositionZero)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect newPosition {0.0F, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), newPosition);
    }

    TEST(CollisionBodyTest, SetPositionNaN)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect newPosition {NAN, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, SetPositionInfinite)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect newPosition {INFINITY, 0.0F};
        body.SetPosition(newPosition);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, UpdatePositionPositive)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect delta {2.0F, 3.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), delta);
    }

    TEST(CollisionBodyTest, UpdatePositionNegative)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect delta {-4.0F, -5.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), delta);
    }

    TEST(CollisionBodyTest, UpdatePositionZero)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect delta {0.0F, 0.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), delta);
    }

    TEST(CollisionBodyTest, UpdatePositionNaN)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect delta {NAN, 0.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, UpdatePositionInfinite)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect delta {INFINITY, 0.0F};
        body.UpdatePosition(delta);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, SetRotationRadiansNormalizesFiniteInputs)
    {
        const RotationCase cases[] = {
            {"Zero",                     0.0F,                    0.0F      },
            {"PositiveHalfPi",           Pi * 0.5F,               Pi * 0.5F },
            {"PositivePi",               Pi,                      Pi        },
            {"PositiveBeforeFullTurn",   TwoPi - 0.25F,           -0.25F    },
            {"PositiveFullTurn",         TwoPi,                   0.0F      },
            {"PositiveMoreThanFullTurn", TwoPi + (Pi * 0.5F),     Pi * 0.5F },
            {"PositiveMultipleTurns",    (TwoPi * 3.0F) + 1.25F,  1.25F     },
            {"NegativeHalfPi",           Pi * -0.5F,              Pi * -0.5F},
            {"NegativePi",               -Pi,                     Pi        },
            {"NegativeFullTurn",         -TwoPi,                  0.0F      },
            {"NegativeMoreThanFullTurn", -TwoPi - (Pi * 0.5F),    Pi * -0.5F},
            {"NegativeMultipleTurns",    (TwoPi * -3.0F) - 1.25F, -1.25F    }
        };

        for (const RotationCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            TestableCollisionBody body;

            body.SetRotation(testCase.Input);

            ExpectRotationNear(body.GetRotation(), testCase.Expected);
        }
    }

    TEST(CollisionBodyTest, GetRotationDegreesConvertsStoredRadiansToDegrees)
    {
        const RotationCase cases[] = {
            {"Zero",                     0.0F,                 0.0F  },
            {"NinetyDegrees",            Pi * 0.5F,            90.0F },
            {"OneHundredEightyDegrees",  Pi,                   180.0F},
            {"NegativeNinetyDegrees",    Pi * 1.5F,            -90.0F},
            {"NegativeFortyFiveDegrees", TwoPi - (Pi * 0.25F), -45.0F}
        };

        for (const RotationCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            TestableCollisionBody body;

            body.SetRotation(testCase.Input);

            ExpectDegreesNear(body.GetRotationDegrees(), testCase.Expected);
        }
    }

    TEST(CollisionBodyTest, SetRotationRadiansIgnoresInvalidInputs)
    {
        const Guch2D::Rotator invalidInputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator invalidInput : invalidInputs)
        {
            SCOPED_TRACE(invalidInput);
            TestableCollisionBody body;
            body.SetRotation(Pi / 3.0F);

            body.SetRotation(invalidInput);

            ExpectRotationNear(body.GetRotation(), Pi / 3.0F);
        }
    }

    TEST(CollisionBodyTest, SetRotationDegreesNormalizesFiniteInputs)
    {
        const DegreeRotationCase cases[] = {
            {"Zero",                     0.0F,             0.0F  },
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

        for (const DegreeRotationCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            TestableCollisionBody body;

            body.SetRotationDegrees(testCase.InputDegrees);

            ExpectDegreesNear(body.GetRotationDegrees(), testCase.ExpectedDegrees);
            ExpectRotationNear(body.GetRotation(), testCase.ExpectedDegrees * DegToRad);
        }
    }

    TEST(CollisionBodyTest, SetRotationDegreesIgnoresInvalidInputs)
    {
        const Guch2D::Rotator invalidInputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator invalidInput : invalidInputs)
        {
            SCOPED_TRACE(invalidInput);
            TestableCollisionBody body;
            body.SetRotationDegrees(45.0F);

            body.SetRotationDegrees(invalidInput);

            ExpectDegreesNear(body.GetRotationDegrees(), 45.0F);
            ExpectRotationNear(body.GetRotation(), 45.0F * DegToRad);
        }
    }

    TEST(CollisionBodyTest, RotateRadiansNormalizesAccumulatedRotation)
    {
        const RotationCase cases[] = {
            {"ZeroDelta",                     0.0F,                           Pi * 0.25F },
            {"PositiveDelta",                 Pi * 0.25F,                     Pi * 0.5F  },
            {"PositivePiDelta",               Pi,                             Pi * -0.75F},
            {"PositiveFullTurnDelta",         TwoPi,                          Pi * 0.25F },
            {"PositiveMoreThanFullTurnDelta", TwoPi + (Pi * 0.25F),           Pi * 0.5F  },
            {"NegativeHalfPiDelta",           Pi * -0.5F,                     Pi * -0.25F},
            {"NegativeFullTurnDelta",         -TwoPi,                         Pi * 0.25F },
            {"NegativeMultipleTurnsDelta",    (TwoPi * -4.0F) - (Pi * 0.25F), 0.0F       }
        };

        for (const RotationCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            TestableCollisionBody body;
            body.SetRotation(Pi * 0.25F);

            body.Rotate(testCase.Input);

            ExpectRotationNear(body.GetRotation(), testCase.Expected);
        }
    }

    TEST(CollisionBodyTest, RotateRadiansIgnoresInvalidDeltas)
    {
        const Guch2D::Rotator invalidInputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator invalidInput : invalidInputs)
        {
            SCOPED_TRACE(invalidInput);
            TestableCollisionBody body;
            body.SetRotation(Pi / 3.0F);

            body.Rotate(invalidInput);

            ExpectRotationNear(body.GetRotation(), Pi / 3.0F);
        }
    }

    TEST(CollisionBodyTest, RotateDegreesNormalizesAccumulatedRotation)
    {
        const DegreeRotationCase cases[] = {
            {"ZeroDelta",                     0.0F,             30.0F },
            {"PositiveDelta",                 90.0F,            120.0F},
            {"PositiveFullTurnDelta",         360.0F,           30.0F },
            {"PositiveMoreThanFullTurnDelta", 390.0F,           60.0F },
            {"PositiveMultipleTurnsDelta",    1080.0F + 45.0F,  75.0F },
            {"NegativeDelta",                 -45.0F,           -15.0F},
            {"NegativeFullTurnDelta",         -360.0F,          30.0F },
            {"NegativeMoreThanFullTurnDelta", -390.0F,          0.0F  },
            {"NegativeMultipleTurnsDelta",    -1080.0F - 45.0F, -15.0F}
        };

        for (const DegreeRotationCase& testCase : cases)
        {
            SCOPED_TRACE(testCase.Name);
            TestableCollisionBody body;
            body.SetRotationDegrees(30.0F);

            body.RotateDegrees(testCase.InputDegrees);

            ExpectDegreesNear(body.GetRotationDegrees(), testCase.ExpectedDegrees);
            ExpectRotationNear(body.GetRotation(), testCase.ExpectedDegrees * DegToRad);
        }
    }

    TEST(CollisionBodyTest, RotateDegreesIgnoresInvalidDeltas)
    {
        const Guch2D::Rotator invalidInputs[] = {NAN, INFINITY, -INFINITY};

        for (const Guch2D::Rotator invalidInput : invalidInputs)
        {
            SCOPED_TRACE(invalidInput);
            TestableCollisionBody body;
            body.SetRotationDegrees(45.0F);

            body.RotateDegrees(invalidInput);

            ExpectDegreesNear(body.GetRotationDegrees(), 45.0F);
            ExpectRotationNear(body.GetRotation(), 45.0F * DegToRad);
        }
    }

    TEST(CollisionBodyTest, SetCollider)
    {
        TestableCollisionBody body;
        const std::shared_ptr<Guch2D::Collider> collider = std::make_shared<Guch2D::CircleCollider>(
            1.0F);
        body.SetCollider(collider);
        EXPECT_EQ(body.GetCollider(), collider);
    }

    TEST(CollisionBodyTest, SetColliderNullptr)
    {
        TestableCollisionBody body;
        const std::shared_ptr<Guch2D::Collider> collider = nullptr;
        body.SetCollider(collider);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, RemoveCollider)
    {
        TestableCollisionBody body;
        const std::shared_ptr<Guch2D::Collider> collider = std::make_shared<Guch2D::CircleCollider>(
            1.0F);
        body.SetCollider(collider);
        body.RemoveCollider();
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(CollisionBodyTest, GetColliderCenterWorldNoCollider)
    {
        const TestableCollisionBody body;
        EXPECT_EQ(body.GetColliderCenterWorld(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, GetColliderCenterWorldWithCollider)
    {
        TestableCollisionBody body;
        constexpr Guch2D::Vect position {1.0F, 2.0F};
        body.SetPosition(position);
        const std::shared_ptr<Guch2D::Collider> collider = std::make_shared<Guch2D::CircleCollider>(
            1.0F);
        constexpr Guch2D::Vect colliderCenter {3.0F, 4.0F};
        collider->SetCenterLocal(colliderCenter);
        body.SetCollider(collider);
        EXPECT_EQ(body.GetColliderCenterWorld(), position + colliderCenter);
    }

    TEST(CollisionBodyTest, GetColliderBordersWorldNoCollider)
    {
        const TestableCollisionBody body;

        EXPECT_EQ(body.GetColliderLeftBorderWorld(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetColliderRightBorderWorld(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetColliderTopBorderWorld(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetColliderBottomBorderWorld(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(CollisionBodyTest, GetColliderBordersWorldWithAABBCollider)
    {
        TestableCollisionBody body;
        body.SetPosition({10.0F, 20.0F});

        const auto collider = std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {1.0F, 2.0F},
                                                                     Guch2D::Vect {3.0F, 4.0F});
        body.SetCollider(collider);

        EXPECT_EQ(body.GetColliderLeftBorderWorld(), Guch2D::Vect(8.0F, 22.0F));
        EXPECT_EQ(body.GetColliderRightBorderWorld(), Guch2D::Vect(14.0F, 22.0F));
        EXPECT_EQ(body.GetColliderTopBorderWorld(), Guch2D::Vect(11.0F, 26.0F));
        EXPECT_EQ(body.GetColliderBottomBorderWorld(), Guch2D::Vect(11.0F, 18.0F));
    }

    TEST(CollisionBodyTest, GetColliderBordersWorldWithCircleCollider)
    {
        TestableCollisionBody body;
        body.SetPosition({-5.0F, 6.0F});

        const auto collider = std::make_shared<Guch2D::CircleCollider>(Guch2D::Vect {2.0F, -1.0F},
                                                                       3.0F);
        body.SetCollider(collider);

        EXPECT_EQ(body.GetColliderLeftBorderWorld(), Guch2D::Vect(-6.0F, 5.0F));
        EXPECT_EQ(body.GetColliderRightBorderWorld(), Guch2D::Vect(0.0F, 5.0F));
        EXPECT_EQ(body.GetColliderTopBorderWorld(), Guch2D::Vect(-3.0F, 8.0F));
        EXPECT_EQ(body.GetColliderBottomBorderWorld(), Guch2D::Vect(-3.0F, 2.0F));
    }

    TEST(CollisionBodyTest, InvokeOnBeginOverlap)
    {
        TestableCollisionBody body;
        bool callbackInvoked = false;
        body.BindOnBeginOverlap(
            [&callbackInvoked](const Guch2D::Collision& /*collision*/) { callbackInvoked = true; });

        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        EXPECT_TRUE(callbackInvoked);
    }

    TEST(CollisionBodyTest, InvokeOnEndOverlap)
    {
        TestableCollisionBody body;
        bool callbackInvoked = false;
        body.BindOnEndOverlap(
            [&callbackInvoked](const Guch2D::Collision& /*collision*/) { callbackInvoked = true; });

        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        EXPECT_TRUE(callbackInvoked);
    }

    TEST(CollisionBodyTest, InvokeOnBeginOverlapNoCallback)
    {
        const TestableCollisionBody body;
        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        SUCCEED();
    }

    TEST(CollisionBodyTest, InvokeOnEndOverlapNoCallback)
    {
        const TestableCollisionBody body;
        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        SUCCEED();
    }

    TEST(CollisionBodyTest, BindOnBeginOverlapReplacesCallback)
    {
        TestableCollisionBody body;
        bool firstCallbackInvoked = false;
        bool secondCallbackInvoked = false;

        body.BindOnBeginOverlap([&firstCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            firstCallbackInvoked = true;
        });
        body.BindOnBeginOverlap([&secondCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            secondCallbackInvoked = true;
        });

        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        EXPECT_FALSE(firstCallbackInvoked);
        EXPECT_TRUE(secondCallbackInvoked);
    }

    TEST(CollsionBodyTest, BindOnEndOverlapReplacesCallback)
    {
        TestableCollisionBody body;
        bool firstCallbackInvoked = false;
        bool secondCallbackInvoked = false;

        body.BindOnEndOverlap([&firstCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            firstCallbackInvoked = true;
        });
        body.BindOnEndOverlap([&secondCallbackInvoked](const Guch2D::Collision& /*collision*/) {
            secondCallbackInvoked = true;
        });

        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        EXPECT_FALSE(firstCallbackInvoked);
        EXPECT_TRUE(secondCallbackInvoked);
    }

    TEST(CollisionBodyTest, BindOnBeginOVverlapNullptr)
    {
        TestableCollisionBody body;
        body.BindOnBeginOverlap(nullptr);

        const Guch2D::Collision collision;
        body.InvokeOnBeginOverlap(collision);
        SUCCEED();
    }

    TEST(CollisionBodyTest, BindOnEndOverlapNullptr)
    {
        TestableCollisionBody body;
        body.BindOnEndOverlap(nullptr);

        const Guch2D::Collision collision;
        body.InvokeOnEndOverlap(collision);
        SUCCEED();
    }

    TEST(ContactPointTest, EqualsWhenAllFieldsMatch)
    {
        const Guch2D::ContactPoint contactPointA {
            {1.0F, 2.0F},
            3.0F,
            4.0F
        };
        const Guch2D::ContactPoint contactPointB {
            {1.0F, 2.0F},
            3.0F,
            4.0F
        };

        EXPECT_TRUE(contactPointA == contactPointB);
    }

    TEST(ContactPointTest, NotEqualsWhenPositionDiffers)
    {
        const Guch2D::ContactPoint contactPointA {
            {1.0F, 2.0F},
            3.0F,
            4.0F
        };
        const Guch2D::ContactPoint contactPointB {
            {2.0F, 2.0F},
            3.0F,
            4.0F
        };

        EXPECT_FALSE(contactPointA == contactPointB);
    }

    TEST(ContactPointTest, NotEqualsWhenAccumulatedNormalImpulseDiffers)
    {
        const Guch2D::ContactPoint contactPointA {
            {1.0F, 2.0F},
            3.0F,
            4.0F
        };
        const Guch2D::ContactPoint contactPointB {
            {1.0F, 2.0F},
            5.0F,
            4.0F
        };

        EXPECT_FALSE(contactPointA == contactPointB);
    }

    TEST(ContactPointTest, NotEqualsWhenAccumulatedTangentImpulseDiffers)
    {
        const Guch2D::ContactPoint contactPointA {
            {1.0F, 2.0F},
            3.0F,
            4.0F
        };
        const Guch2D::ContactPoint contactPointB {
            {1.0F, 2.0F},
            3.0F,
            5.0F
        };

        EXPECT_FALSE(contactPointA == contactPointB);
    }

    TEST(CollisionStrcutTest, EqualsOperator)
    {
        Guch2D::Collision collisionA;
        Guch2D::Collision collisionB;

        EXPECT_TRUE(collisionA == collisionB);

        auto bodyA = std::make_shared<TestableCollisionBody>();
        collisionA.BodyA = bodyA;
        EXPECT_FALSE(collisionA == collisionB);

        collisionB.BodyA = bodyA;
        EXPECT_TRUE(collisionA == collisionB);

        auto bodyB = std::make_shared<TestableCollisionBody>();
        collisionA.BodyB = bodyB;
        EXPECT_FALSE(collisionA == collisionB);

        collisionB.BodyB = bodyB;
        EXPECT_TRUE(collisionA == collisionB);
    }

}   // namespace
