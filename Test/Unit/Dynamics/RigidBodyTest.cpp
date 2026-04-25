#include "Dynamics/RigidBody.hpp"
#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <gtest/gtest.h>

namespace
{
    TEST(RigidBodyTest, DefaultConstructor)
    {
        const Guch2D::RigidBody body;
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(RigidBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position(5.0F, -3.0F);
        const Guch2D::RigidBody body(position);
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(RigidBodyTest, PositionAndMassConstructor)
    {
        constexpr Guch2D::Vect position(2.0F, 4.0F);
        constexpr float mass = 10.0F;
        const Guch2D::RigidBody body(position, mass);
        EXPECT_EQ(body.GetMass(), mass);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(RigidBodyTest, SetMassPositive)
    {
        Guch2D::RigidBody body;
        body.SetMass(15.0F);
        EXPECT_EQ(body.GetMass(), 15.0F);
    }

    TEST(RigidBodyTest, SetMassNegative)
    {
        Guch2D::RigidBody body;
        body.SetMass(-5.0F);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassZero)
    {
        Guch2D::RigidBody body;
        body.SetMass(0.0F);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassNaN)
    {
        Guch2D::RigidBody body;
        body.SetMass(NAN);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassInfinity)
    {
        Guch2D::RigidBody body;
        body.SetMass(INFINITY);
        EXPECT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetDensityWithoutColliderKeepsMassZero)
    {
        Guch2D::RigidBody body;

        body.SetDensity(4.5F);

        EXPECT_FLOAT_EQ(body.GetDensity(), 4.5F);
        EXPECT_FLOAT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetDensityWithCircleColliderComputesMassFromArea)
    {
        Guch2D::RigidBody body;
        auto collider = std::make_shared<Guch2D::CircleCollider>(2.0F);
        body.SetCollider(collider);

        body.SetDensity(3.0F);

        EXPECT_FLOAT_EQ(body.GetDensity(), 3.0F);
        EXPECT_NEAR(body.GetMass(), 3.0F * collider->GetArea(), 1.0e-5F);
    }

    TEST(RigidBodyTest, MassFromDensityUpdatesWhenCircleRadiusChanges)
    {
        Guch2D::RigidBody body;
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
        Guch2D::RigidBody body;
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
        Guch2D::RigidBody body;
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
        Guch2D::RigidBody body;
        body.SetCollider(std::make_shared<Guch2D::CircleCollider>(2.0F));
        body.SetDensity(2.0F);

        EXPECT_GT(body.GetMass(), 0.0F);

        body.RemoveCollider();
        EXPECT_FLOAT_EQ(body.GetMass(), 0.0F);
    }

    TEST(RigidBodyTest, SetMassSwitchesToManualModeAndClearsDensity)
    {
        Guch2D::RigidBody body;
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
        Guch2D::RigidBody body;
        auto collider = std::make_shared<Guch2D::CircleCollider>(2.0F);
        body.SetCollider(collider);
        body.SetMass(9.0F);
        body.SetDensity(4.0F);

        EXPECT_NE(body.GetMass(), 9.0F);

        body.SetDensity(0.0F);
        EXPECT_FLOAT_EQ(body.GetDensity(), 0.0F);
        EXPECT_FLOAT_EQ(body.GetMass(), 9.0F);
    }
}   // namespace
