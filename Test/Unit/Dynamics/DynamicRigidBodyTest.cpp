#include "Dynamics/DynamicRigidBody.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(DynamicRigidBodyTest, DefaultConstructor)
    {
        const Guch2D::DynamicRigidBody body;
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetForce(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetGravityScale(), Guch2D::DynamicRigidBody::DefaultGravityScale);
        EXPECT_EQ(body.GetLinearDamping(), Guch2D::DynamicRigidBody::DefaultLinearDamping);
    }

    TEST(DynamicRigidBodyTest, PositionConstructor)
    {
        constexpr auto position = Guch2D::Vect(0.0F, 0.0F);
        const Guch2D::DynamicRigidBody body(position);
        EXPECT_EQ(body.GetMass(), 0.0F);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetForce(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetGravityScale(), Guch2D::DynamicRigidBody::DefaultGravityScale);
        EXPECT_EQ(body.GetLinearDamping(), Guch2D::DynamicRigidBody::DefaultLinearDamping);
    }

    TEST(DymanicRigidBodyTest, PositionAndMassConstructor)
    {
        constexpr auto position = Guch2D::Vect(0.0F, 0.0F);
        constexpr float mass = 5.0F;
        const Guch2D::DynamicRigidBody body(position, mass);
        EXPECT_EQ(body.GetMass(), mass);
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
        EXPECT_EQ(body.GetForce(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetGravityScale(), Guch2D::DynamicRigidBody::DefaultGravityScale);
        EXPECT_EQ(body.GetLinearDamping(), Guch2D::DynamicRigidBody::DefaultLinearDamping);
    }

    TEST(DynamicRigidBodyTest, SetForcePositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect force(3.0F, 4.0F);
        body.SetForce(force);
        EXPECT_EQ(body.GetForce(), force);
    }

    TEST(DynamicRigidBodyTest, SetForceNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect force(-2.0F, -5.0F);
        body.SetForce(force);
        EXPECT_EQ(body.GetForce(), force);
    }

    TEST(DymanicRigidBodyTest, SetForceZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect force(0.0F, 0.0F);
        body.SetForce(force);
        EXPECT_EQ(body.GetForce(), force);
    }

    TEST(DymanicRigidBodyTest, SetForceNaN)
    {
        Guch2D::DynamicRigidBody body;
        body.SetForce({NAN, 2.0F});
        EXPECT_EQ(body.GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, SetForceInfinity)
    {
        Guch2D::DynamicRigidBody body;
        body.SetForce({INFINITY, 2.0F});
        EXPECT_EQ(body.GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, AddForcePositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialForce(1.0F, 1.0F);
        body.SetForce(initialForce);
        constexpr Guch2D::Vect additionalForce(2.0F, 3.0F);
        body.AddForce(additionalForce);
        EXPECT_EQ(body.GetForce(), initialForce + additionalForce);
    }

    TEST(DynamicRigidBodyTest, AddForceNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialForce(-1.0F, -1.0F);
        body.SetForce(initialForce);
        constexpr Guch2D::Vect additionalForce(-2.0F, -3.0F);
        body.AddForce(additionalForce);
        EXPECT_EQ(body.GetForce(), initialForce + additionalForce);
    }

    TEST(DynamicRigidBodyTest, AddForceZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialForce(2.0F, -3.0F);
        body.SetForce(initialForce);
        constexpr Guch2D::Vect additionalForce(0.0F, 0.0F);
        body.AddForce(additionalForce);
        EXPECT_EQ(body.GetForce(), initialForce + additionalForce);
    }

    TEST(DynamicRigidBodyTest, AddForceNaN)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialForce(2.0F, 3.0F);
        body.SetForce(initialForce);
        body.AddForce({NAN, 1.0F});
        EXPECT_EQ(body.GetForce(), initialForce);
    }

    TEST(DynamicRigidBodyTest, ResetForce)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect force(5.0F, -7.0F);
        body.SetForce(force);
        body.ResetForce();
        EXPECT_EQ(body.GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, SetAccelerationPositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect acceleration(3.0F, 4.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(DynamicRigidBodyTest, SetAccelerationNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect acceleration(-2.0F, -5.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(DynamicRigidBodyTest, SetAccelerationZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect acceleration(0.0F, 0.0F);
        body.SetAcceleration(acceleration);
        EXPECT_EQ(body.GetAcceleration(), acceleration);
    }

    TEST(DynamicRigidBodyTest, SetAccelerationNaN)
    {
        Guch2D::DynamicRigidBody body;
        body.SetAcceleration({NAN, 2.0F});
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, SetAccelerationInfinity)
    {
        Guch2D::DynamicRigidBody body;
        body.SetAcceleration({INFINITY, 2.0F});
        EXPECT_EQ(body.GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, SetVelocityPositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect velocity(3.0F, 4.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(DynamicRigidBodyTest, SetVelocityNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect velocity(-2.0F, -5.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(DynamicRigidBodyTest, SetVelocityZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect velocity(0.0F, 0.0F);
        body.SetVelocity(velocity);
        EXPECT_EQ(body.GetVelocity(), velocity);
    }

    TEST(DymaincRigidBodyTest, SetVelocityNaN)
    {
        Guch2D::DynamicRigidBody body;
        body.SetVelocity({NAN, 2.0F});
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, SetVelocityInfinity)
    {
        Guch2D::DynamicRigidBody body;
        body.SetVelocity({INFINITY, 2.0F});
        EXPECT_EQ(body.GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicRigidBodyTest, AddVelocityPositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(1.0F, 1.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(2.0F, 3.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(DynamicRigidBodyTest, AddVelocityNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(-1.0F, -1.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(-2.0F, -3.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(DynamicRigidBodyTest, AddVelocityZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, -3.0F);
        body.SetVelocity(initialVelocity);
        constexpr Guch2D::Vect additionalVelocity(0.0F, 0.0F);
        body.AddVelocity(additionalVelocity);
        EXPECT_EQ(body.GetVelocity(), initialVelocity + additionalVelocity);
    }

    TEST(DynamicRigidBodyTest, AddVelocityNaN)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, 3.0F);
        body.SetVelocity(initialVelocity);
        body.AddVelocity({NAN, 1.0F});
        EXPECT_EQ(body.GetVelocity(), initialVelocity);
    }

    TEST(DynamicRigidBodyTest, AddVelocityInfinity)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect initialVelocity(2.0F, 3.0F);
        body.SetVelocity(initialVelocity);
        body.AddVelocity({INFINITY, 1.0F});
        EXPECT_EQ(body.GetVelocity(), initialVelocity);
    }

    TEST(DynamicRigidBodyTest, SetGravityScalePositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect scale(2.0F, 3.0F);
        body.SetGravityScale(scale);
        EXPECT_EQ(body.GetGravityScale(), scale);
    }

    TEST(DymanicRigidBodyTest, SetGravityScaleNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect scale(-1.0F, -2.0F);
        body.SetGravityScale(scale);
        EXPECT_EQ(body.GetGravityScale(), scale);
    }

    TEST(DynamicRigidBodyTest, SetGravityScaleZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect scale(0.0F, 0.0F);
        body.SetGravityScale(scale);
        EXPECT_EQ(body.GetGravityScale(), scale);
    }

    TEST(DynamicRigidBodyTest, SetGravityScaleNaN)
    {
        Guch2D::DynamicRigidBody body;
        body.SetGravityScale({NAN, 1.0F});
        EXPECT_EQ(body.GetGravityScale(), Guch2D::DynamicRigidBody::DefaultGravityScale);
    }

    TEST(DynamicRigidBodyTest, SetGravityScaleInfinity)
    {
        Guch2D::DynamicRigidBody body;
        body.SetGravityScale({INFINITY, 1.0F});
        EXPECT_EQ(body.GetGravityScale(), Guch2D::DynamicRigidBody::DefaultGravityScale);
    }

    TEST(DynamicRigidBodyTest, SetLinearDampingPositive)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect damping(0.1F, 0.2F);
        body.SetLinearDamping(damping);
        EXPECT_EQ(body.GetLinearDamping(), damping);
    }

    TEST(DynamicRigidBodyTest, SetLinearDampingNegative)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect damping(-0.1F, -0.2F);
        body.SetLinearDamping(damping);
        EXPECT_EQ(body.GetLinearDamping(), damping);
    }

    TEST(DymanicRigidBodyTest, SetLinearDampingZero)
    {
        Guch2D::DynamicRigidBody body;
        constexpr Guch2D::Vect damping(0.0F, 0.0F);
        body.SetLinearDamping(damping);
        EXPECT_EQ(body.GetLinearDamping(), damping);
    }

    TEST(DynamicRigidBodyTest, SetLinearDampingNaN)
    {
        Guch2D::DynamicRigidBody body;
        body.SetLinearDamping({NAN, 0.1F});
        EXPECT_EQ(body.GetLinearDamping(), Guch2D::DynamicRigidBody::DefaultLinearDamping);
    }

    TEST(DynamicRigidBodyTest, SetLinearDampingInfinity)
    {
        Guch2D::DynamicRigidBody body;
        body.SetLinearDamping({INFINITY, 0.1F});
        EXPECT_EQ(body.GetLinearDamping(), Guch2D::DynamicRigidBody::DefaultLinearDamping);
    }

    TEST(DynamicRigidBodyTest, DefaultSimulatePhysicsTrue)
    {
        Guch2D::DynamicRigidBody body;
        EXPECT_TRUE(body.GetSimulatePhysics());
    }

    TEST(DynamicRigidBodyTest, SetSimulatePhysicsTrue)
    {
        Guch2D::DynamicRigidBody body;
        body.SetSimulatePhysics(true);
        EXPECT_TRUE(body.GetSimulatePhysics());
    }

    TEST(DynamicRigidBodyTest, SetSimulatePhysicsFalse)
    {
        Guch2D::DynamicRigidBody body;
        body.SetSimulatePhysics(false);
        EXPECT_FALSE(body.GetSimulatePhysics());
    }
}   // namespace
