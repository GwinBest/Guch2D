#include "Dynamics/KinematicBody.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace
{
    TEST(KinematicBodyTest, DefaultConstructor)
    {
        const Guch2D::KinematicBody body;
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position(5.0F, -3.0F);
        const Guch2D::KinematicBody body(position);
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, PositionAndMassConstructor)
    {
        constexpr Guch2D::Vect position(2.0F, 4.0F);
        constexpr float mass = 10.0F;
        const Guch2D::KinematicBody body(position, mass);
        EXPECT_EQ(body.GetMass(), mass);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, SetAccelerationPositive)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect acceleration(3.0F, 4.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(KinematicBodyTest, SetAccelerationNegative)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect acceleration(-2.0F, -5.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(KinematicBodyTest, SetAccelerationZero)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect acceleration(0.0F, 0.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(KinematicBodyTest, SetAccelerationNaN)
    {
        Guch2D::KinematicBody body;
        body.SetAcceleration({NAN, 2.0F});
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, SetAccelerationInfinity)
    {
        Guch2D::KinematicBody body;
        body.SetAcceleration({INFINITY, 2.0F});
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, SetVelocityPositive)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect velocity(3.0F, 4.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(KinematicBodyTest, SetVelocityNegative)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect velocity(-2.0F, -5.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(KinematicBodyTest, SetVelocityZero)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect velocity(0.0F, 0.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(KinematicBodyTest, SetVelocityNaN)
    {
        Guch2D::KinematicBody body;
        body.SetVelocity({NAN, 2.0F});
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, SetVelocityInfinity)
    {
        Guch2D::KinematicBody body;
        body.SetVelocity({INFINITY, 2.0F});
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicBodyTest, AddVelocityPositive)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect initialVelocity(1.0F, 1.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(2.0F, 3.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicBodyTest, AddVelocityNegative)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect initialVelocity(-1.0F, -1.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(-2.0F, -3.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicBodyTest, AddVelocityZero)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, -3.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(0.0F, 0.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicBodyTest, AddVelocityNaN)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, 3.0F);
        body.SetVelocity(initialVelocity);
        body.AddVelocity({NAN, 1.0F});
        EXPECT_EQ(body.GetVelocity(), initialVelocity);
    }

    TEST(KinematicBodyTest, AddVelocityInfinity)
    {
        Guch2D::KinematicBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, 3.0F);
        body.SetVelocity(initialVelocity);
        body.AddVelocity({INFINITY, 1.0F});
        EXPECT_EQ(body.GetVelocity(), initialVelocity);
    }
}   // namespace
