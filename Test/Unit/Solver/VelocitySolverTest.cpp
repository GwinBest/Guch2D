#include "Solver/VelocitySolver.hpp"

#include <gtest/gtest.h>

#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/StaticRigidBody.hpp"

namespace
{
    TEST(VelocitySolverTest, StopsDynamicBodyOnStaticBodyContactWithoutFriction)
    {
        const auto dynamicBody = std::make_shared<Guch2D::DynamicRigidBody>();
        dynamicBody->SetMass(2.0F);
        dynamicBody->SetVelocity({1.0F, -5.0F});

        const auto staticBody = std::make_shared<Guch2D::StaticRigidBody>();

        Guch2D::Collision collision;
        collision.BodyA = dynamicBody;
        collision.BodyB = staticBody;
        collision.Points.Normal = {0.0F, 1.0F};
        collision.Points.HasCollision = true;

        Guch2D::VelocitySolver solver;
        solver.Solve({collision});

        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().x, 1.0F);
        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().y, 0.0F);
    }

    TEST(VelocitySolverTest, KeepsVelocityWhenBodiesAreSeparating)
    {
        const auto dynamicBody = std::make_shared<Guch2D::DynamicRigidBody>();
        dynamicBody->SetMass(2.0F);
        dynamicBody->SetVelocity({0.0F, 3.0F});

        const auto staticBody = std::make_shared<Guch2D::StaticRigidBody>();

        Guch2D::Collision collision;
        collision.BodyA = dynamicBody;
        collision.BodyB = staticBody;
        collision.Points.Normal = {0.0F, 1.0F};
        collision.Points.HasCollision = true;

        Guch2D::VelocitySolver solver;
        solver.Solve({collision});

        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().x, 0.0F);
        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().y, 3.0F);
    }

    TEST(VelocitySolverTest, TangentialImpulseCancelsTangentialVelocityWithHighStaticFriction)
    {
        const auto dynamicBody = std::make_shared<Guch2D::DynamicRigidBody>();
        dynamicBody->SetMass(2.0F);
        dynamicBody->SetVelocity({1.0F, -5.0F});
        dynamicBody->SetStaticFriction(1.0F);
        dynamicBody->SetDynamicFriction(1.0F);

        const auto staticBody = std::make_shared<Guch2D::StaticRigidBody>();
        staticBody->SetStaticFriction(1.0F);
        staticBody->SetDynamicFriction(1.0F);

        Guch2D::Collision collision;
        collision.BodyA = dynamicBody;
        collision.BodyB = staticBody;
        collision.Points.Normal = {0.0F, 1.0F};
        collision.Points.HasCollision = true;

        Guch2D::VelocitySolver solver;
        solver.Solve({collision});

        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().x, 0.0F);
        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().y, 0.0F);
    }

    TEST(VelocitySolverTest, TangentialImpulseUsesDynamicFrictionClampWhenStaticFrictionIsZero)
    {
        const auto dynamicBody = std::make_shared<Guch2D::DynamicRigidBody>();
        dynamicBody->SetMass(2.0F);
        dynamicBody->SetVelocity({1.0F, -5.0F});
        dynamicBody->SetStaticFriction(0.0F);
        dynamicBody->SetDynamicFriction(0.1F);

        const auto staticBody = std::make_shared<Guch2D::StaticRigidBody>();
        staticBody->SetStaticFriction(0.0F);
        staticBody->SetDynamicFriction(0.1F);

        Guch2D::Collision collision;
        collision.BodyA = dynamicBody;
        collision.BodyB = staticBody;
        collision.Points.Normal = {0.0F, 1.0F};
        collision.Points.HasCollision = true;

        Guch2D::VelocitySolver solver;
        solver.Solve({collision});

        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().x, 0.5F);
        EXPECT_FLOAT_EQ(dynamicBody->GetVelocity().y, 0.0F);
    }
}   // namespace
