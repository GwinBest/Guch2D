#include "Dynamics/RigidBody.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <numbers>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"

namespace
{
    class TestableRigidBody final : public Guch2D::RigidBody
    {
    public:
        using Guch2D::RigidBody::RigidBody;
    };

    TEST(RigidBodyTest, DefaultConstructor)
    {
        const TestableRigidBody body;
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(RigidBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position(5.0F, -3.0F);
        const TestableRigidBody body(position);
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(RigidBodyTest, PositionAndMassConstructor)
    {
        constexpr Guch2D::Vect position(2.0F, 4.0F);
        constexpr float mass = 10.0F;
        const TestableRigidBody body(position, mass);
        EXPECT_EQ(body.GetMass(), mass);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(RigidBodyTest, SetMassPositive)
    {
        TestableRigidBody body;
        body.SetMass(15.0F);
        EXPECT_EQ(body.GetMass(), 15.0F);
    }

    TEST(RigidBodyTest, SetMassNegative)
    {
        TestableRigidBody body;
        body.SetMass(-5.0F);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassZero)
    {
        TestableRigidBody body;
        body.SetMass(0.0F);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassNaN)
    {
        TestableRigidBody body;
        body.SetMass(NAN);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassInfinity)
    {
        TestableRigidBody body;
        body.SetMass(INFINITY);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetDensityWithoutColliderKeepsMassZero)
    {
        TestableRigidBody body;

        body.SetDensity(4.5F);

        EXPECT_FLOAT_EQ(body.GetDensity(), 4.5F);
        EXPECT_FLOAT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetDensityWithCircleColliderComputesMassFromArea)
    {
        TestableRigidBody body;
        auto collider = std::make_shared<Guch2D::CircleCollider>(2.0F);
        body.SetCollider(collider);

        body.SetDensity(3.0F);

        EXPECT_FLOAT_EQ(body.GetDensity(), 3.0F);
        EXPECT_NEAR(body.GetMass(), 3.0F * collider->GetArea(), 1.0e-5F);
    }

    TEST(RigidBodyTest, MassFromDensityUpdatesWhenCircleRadiusChanges)
    {
        TestableRigidBody body;
        auto collider = std::make_shared<Guch2D::CircleCollider>(1.0F);
        body.SetCollider(collider);
        body.SetDensity(2.0F);

        const float initialMass = body.GetMass();
        collider->SetRadius(3.0F);

        EXPECT_NEAR(initialMass, 2.0F * std::numbers::pi_v<float>, 1.0e-5F);
        EXPECT_NEAR(body.GetMass(), 2.0F * collider->GetArea(), 1.0e-5F);
    }

    TEST(RigidBodyTest, MassFromDensityUpdatesWhenAABBExtentChanges)
    {
        TestableRigidBody body;
        auto collider = std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {1.0F, 2.0F});
        body.SetCollider(collider);
        body.SetDensity(1.25F);

        const float initialMass = body.GetMass();
        collider->SetExtent(Guch2D::Vect {3.0F, 4.0F});

        EXPECT_FLOAT_EQ(initialMass, 1.25F * 8.0F);
        EXPECT_FLOAT_EQ(body.GetMass(), 1.25F * collider->GetArea());
    }

    TEST(RigidBodyTest, MassFromDensityUpdatesWhenColliderIsReplaced)
    {
        TestableRigidBody body;
        body.SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
        body.SetDensity(2.0F);

        const float circleMass = body.GetMass();

        const auto aabbCollider = std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {2.0F, 3.0F});
        body.SetCollider(aabbCollider);

        EXPECT_NEAR(circleMass, 2.0F * std::numbers::pi_v<float>, 1.0e-5F);
        EXPECT_FLOAT_EQ(body.GetMass(), 2.0F * aabbCollider->GetArea());
    }

    TEST(RigidBodyTest, MassFromDensityBecomesZeroWhenColliderRemoved)
    {
        TestableRigidBody body;
        body.SetCollider(std::make_shared<Guch2D::CircleCollider>(2.0F));
        body.SetDensity(2.0F);

        EXPECT_GT(body.GetMass(), 0.0F);

        body.RemoveCollider();
        EXPECT_FLOAT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassSwitchesToManualModeAndClearsDensity)
    {
        TestableRigidBody body;
        auto collider = std::make_shared<Guch2D::CircleCollider>(2.0F);
        body.SetCollider(collider);
        body.SetDensity(5.0F);

        body.SetMass(7.0F);
        collider->SetRadius(5.0F);

        EXPECT_FLOAT_EQ(body.GetDensity(), 0.0F);
        EXPECT_FLOAT_EQ(body.GetMass(), 7.0F);
    }

    TEST(RigidBodyTest, SetInvalidDensityDisablesDensityModeAndRestoresManualMass)
    {
        TestableRigidBody body;
        auto collider = std::make_shared<Guch2D::CircleCollider>(2.0F);
        body.SetCollider(collider);
        body.SetMass(9.0F);
        body.SetDensity(4.0F);

        EXPECT_NE(body.GetMass(), 9.0F);

        body.SetDensity(0.0F);
        EXPECT_FLOAT_EQ(body.GetDensity(), 0.0F);
        EXPECT_FLOAT_EQ(body.GetMass(), 9.0F);
    }

    TEST(RigidBodyTest, DefaultBouncinessIsZero)
    {
        const TestableRigidBody body;
        EXPECT_FLOAT_EQ(body.GetBounciness(), 0.0F);
    }

    TEST(RigidBodyTest, SetBouncinessWithinRange)
    {
        TestableRigidBody body;
        body.SetBounciness(0.65F);
        EXPECT_FLOAT_EQ(body.GetBounciness(), 0.65F);
    }

    TEST(RigidBodyTest, SetBouncinessBelowRangeClampsToZero)
    {
        TestableRigidBody body;
        body.SetBounciness(-0.25F);
        EXPECT_FLOAT_EQ(body.GetBounciness(), 0.0F);
    }

    TEST(RigidBodyTest, SetBouncinessAboveRangeClampsToOne)
    {
        TestableRigidBody body;
        body.SetBounciness(1.75F);
        EXPECT_FLOAT_EQ(body.GetBounciness(), 1.0F);
    }

    TEST(RigidBodyTest, SetBouncinessNaNResetsToZero)
    {
        TestableRigidBody body;
        body.SetBounciness(0.5F);
        body.SetBounciness(NAN);
        EXPECT_FLOAT_EQ(body.GetBounciness(), 0.0F);
    }

    TEST(RigidBodyTest, SetBouncinessInfinityResetsToZero)
    {
        TestableRigidBody body;
        body.SetBounciness(0.5F);
        body.SetBounciness(INFINITY);
        EXPECT_FLOAT_EQ(body.GetBounciness(), 0.0F);
    }

    TEST(RigidBodyTest, DefaultStaticFrictionIsZero)
    {
        const TestableRigidBody body;
        EXPECT_FLOAT_EQ(body.GetStaticFriction(), 0.0F);
    }

    TEST(RigidBodyTest, SetStaticFrictionWithinRange)
    {
        TestableRigidBody body;
        body.SetStaticFriction(0.65F);
        EXPECT_FLOAT_EQ(body.GetStaticFriction(), 0.65F);
    }

    TEST(RigidBodyTest, SetStaticFrictionBelowRangeClampsToZero)
    {
        TestableRigidBody body;
        body.SetStaticFriction(-0.25F);
        EXPECT_FLOAT_EQ(body.GetStaticFriction(), 0.0F);
    }

    TEST(RigidBodyTest, SetStaticFrictionAboveRangeClampsToOne)
    {
        TestableRigidBody body;
        body.SetStaticFriction(1.75F);
        EXPECT_FLOAT_EQ(body.GetStaticFriction(), 1.0F);
    }

    TEST(RigidBodyTest, SetStaticFrictionNaNResetsToZero)
    {
        TestableRigidBody body;
        body.SetStaticFriction(0.5F);
        body.SetStaticFriction(NAN);
        EXPECT_FLOAT_EQ(body.GetStaticFriction(), 0.0F);
    }

    TEST(RigidBodyTest, SetStaticFrictionInfinityResetsToZero)
    {
        TestableRigidBody body;
        body.SetStaticFriction(0.5F);
        body.SetStaticFriction(INFINITY);
        EXPECT_FLOAT_EQ(body.GetStaticFriction(), 0.0F);
    }

    TEST(RigidBodyTest, DefaultDynamicFrictionIsZero)
    {
        const TestableRigidBody body;
        EXPECT_FLOAT_EQ(body.GetDynamicFriction(), 0.0F);
    }

    TEST(RigidBodyTest, SetDynamicFrictionWithinRange)
    {
        TestableRigidBody body;
        body.SetDynamicFriction(0.4F);
        EXPECT_FLOAT_EQ(body.GetDynamicFriction(), 0.4F);
    }

    TEST(RigidBodyTest, SetDynamicFrictionBelowRangeClampsToZero)
    {
        TestableRigidBody body;
        body.SetDynamicFriction(-0.25F);
        EXPECT_FLOAT_EQ(body.GetDynamicFriction(), 0.0F);
    }

    TEST(RigidBodyTest, SetDynamicFrictionAboveRangeClampsToOne)
    {
        TestableRigidBody body;
        body.SetDynamicFriction(1.75F);
        EXPECT_FLOAT_EQ(body.GetDynamicFriction(), 1.0F);
    }

    TEST(RigidBodyTest, SetDynamicFrictionNaNResetsToZero)
    {
        TestableRigidBody body;
        body.SetDynamicFriction(0.5F);
        body.SetDynamicFriction(NAN);
        EXPECT_FLOAT_EQ(body.GetDynamicFriction(), 0.0F);
    }

    TEST(RigidBodyTest, SetDynamicFrictionInfinityResetsToZero)
    {
        TestableRigidBody body;
        body.SetDynamicFriction(0.5F);
        body.SetDynamicFriction(INFINITY);
        EXPECT_FLOAT_EQ(body.GetDynamicFriction(), 0.0F);
    }
}   // namespace
