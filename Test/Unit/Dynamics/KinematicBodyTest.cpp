#include <cmath>
#include <gtest/gtest.h>

#include "Dynamics/KinematicRigidBody.hpp"

namespace
{
    TEST(KinematicRigidBodyTest, DefaultConstructor)
    {
        const Guch2D::KinematicRigidBody body;
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularDamping(), Guch2D::KinematicRigidBody::DefaultAngularDamping);
    }

    TEST(KinematicRigidBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position(5.0F, -3.0F);
        const Guch2D::KinematicRigidBody body(position);
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularDamping(), Guch2D::KinematicRigidBody::DefaultAngularDamping);
    }

    TEST(KinematicRigidBodyTest, PositionAndMassConstructor)
    {
        constexpr Guch2D::Vect position(2.0F, 4.0F);
        const Guch2D::KinematicRigidBody body(position);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAngularDamping(), Guch2D::KinematicRigidBody::DefaultAngularDamping);
    }

    TEST(KinematicRigidBodyTest, SetAccelerationPositive)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect acceleration(3.0F, 4.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(KinematicRigidBodyTest, SetAccelerationNegative)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect acceleration(-2.0F, -5.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(KinematicRigidBodyTest, SetAccelerationZero)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect acceleration(0.0F, 0.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(KinematicRigidBodyTest, SetAccelerationNaN)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAcceleration({NAN, 2.0F});
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, SetAccelerationInfinity)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAcceleration({INFINITY, 2.0F});
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, SetVelocityPositive)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect velocity(3.0F, 4.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(KinematicRigidBodyTest, SetVelocityNegative)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect velocity(-2.0F, -5.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(KinematicRigidBodyTest, SetVelocityZero)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect velocity(0.0F, 0.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(KinematicRigidBodyTest, SetVelocityNaN)
    {
        Guch2D::KinematicRigidBody body;
        body.SetVelocity({NAN, 2.0F});
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, SetVelocityInfinity)
    {
        Guch2D::KinematicRigidBody body;
        body.SetVelocity({INFINITY, 2.0F});
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, AddVelocityPositive)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(1.0F, 1.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(2.0F, 3.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicRigidBodyTest, AddVelocityNegative)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(-1.0F, -1.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(-2.0F, -3.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicRigidBodyTest, AddVelocityZero)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, -3.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(0.0F, 0.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicRigidBodyTest, AddVelocityNaN)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, 3.0F);
        body.SetVelocity(initialVelocity);
        body.AddVelocity({NAN, 1.0F});
        EXPECT_EQ(body.GetVelocity(), initialVelocity);
    }

    TEST(KinematicRigidBodyTest, AddVelocityInfinity)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, 3.0F);
        body.SetVelocity(initialVelocity);
        body.AddVelocity({INFINITY, 1.0F});
        EXPECT_EQ(body.GetVelocity(), initialVelocity);
    }

    TEST(KinematicRigidBodyTest, SetAngularAcceleration)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect acceleration(3.0F, -4.0F);
        body.SetAngularAcceleration(acceleration);
        EXPECT_EQ(body.GetAngularAcceleration(), acceleration);
    }

    TEST(KinematicRigidBodyTest, SetAngularAccelerationNaN)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAngularAcceleration({NAN, 2.0F});
        EXPECT_EQ(body.GetAngularAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, SetAngularAccelerationInfinity)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAngularAcceleration({INFINITY, 2.0F});
        EXPECT_EQ(body.GetAngularAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, SetAngularVelocity)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect velocity(1.5F, -2.5F);
        body.SetAngularVelocity(velocity);
        EXPECT_EQ(body.GetAngularVelocity(), velocity);
    }

    TEST(KinematicRigidBodyTest, SetAngularVelocityNaN)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAngularVelocity({NAN, 2.0F});
        EXPECT_EQ(body.GetAngularVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, SetAngularVelocityInfinity)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAngularVelocity({INFINITY, 2.0F});
        EXPECT_EQ(body.GetAngularVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(KinematicRigidBodyTest, AddAngularVelocity)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(1.0F, -2.0F);
        constexpr Guch2D::Vect additionalVelocity(3.0F, 4.0F);
        body.SetAngularVelocity(initialVelocity);
        body.AddAngularVelocity(additionalVelocity);
        EXPECT_EQ(body.GetAngularVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(KinematicRigidBodyTest, AddAngularVelocityNaN)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(1.0F, -2.0F);
        body.SetAngularVelocity(initialVelocity);
        body.AddAngularVelocity({NAN, 1.0F});
        EXPECT_EQ(body.GetAngularVelocity(), initialVelocity);
    }

    TEST(KinematicRigidBodyTest, AddAngularVelocityInfinity)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(1.0F, -2.0F);
        body.SetAngularVelocity(initialVelocity);
        body.AddAngularVelocity({INFINITY, 1.0F});
        EXPECT_EQ(body.GetAngularVelocity(), initialVelocity);
    }

    TEST(KinematicRigidBodyTest, SetAngularDamping)
    {
        Guch2D::KinematicRigidBody body;
        constexpr Guch2D::Vect damping(0.3F, 0.4F);
        body.SetAngularDamping(damping);
        EXPECT_EQ(body.GetAngularDamping(), damping);
    }

    TEST(KinematicRigidBodyTest, SetAngularDampingNaN)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAngularDamping({NAN, 0.1F});
        EXPECT_EQ(body.GetAngularDamping(), Guch2D::KinematicRigidBody::DefaultAngularDamping);
    }

    TEST(KinematicRigidBodyTest, SetAngularDampingInfinity)
    {
        Guch2D::KinematicRigidBody body;
        body.SetAngularDamping({INFINITY, 0.1F});
        EXPECT_EQ(body.GetAngularDamping(), Guch2D::KinematicRigidBody::DefaultAngularDamping);
    }
}   // namespace
