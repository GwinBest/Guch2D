#include "Dynamics/RigidBody.hpp"

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
}   // namespace
