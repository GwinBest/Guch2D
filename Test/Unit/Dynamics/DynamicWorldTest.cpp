#include "Dynamics/DynamicWorld.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(DynamicWorldTest, DefaultConstructor)
    {
        const Guch2D::DynamicWorld world;
        EXPECT_EQ(world.GetGravity(), Guch2D::DynamicWorld::DefaultGravity);
        EXPECT_EQ(world.GetTimeStep(), Guch2D::CollisionWorld::DefaultTimeStep);
    }

    TEST(DynamicWorldTest, SetGravityPositive)
    {
        Guch2D::DynamicWorld world;
        constexpr Guch2D::Vect gravity(0.0F, 9.81F);
        world.SetGravity(gravity);
        EXPECT_EQ(world.GetGravity(), gravity);
    }

    TEST(DynamicWorldTest, SetGravityNegative)
    {
        Guch2D::DynamicWorld world;
        constexpr Guch2D::Vect gravity(0.0F, -9.81F);
        world.SetGravity(gravity);
        EXPECT_EQ(world.GetGravity(), gravity);
    }

    TEST(DynamicWorldTest, SetGravityZero)
    {
        Guch2D::DynamicWorld world;
        constexpr Guch2D::Vect gravity(0.0F, 0.0F);
        world.SetGravity(gravity);
        EXPECT_EQ(world.GetGravity(), gravity);
    }

    TEST(DynamicWorldTest, SetGravityNaN)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, NAN});
        EXPECT_EQ(world.GetGravity(), Guch2D::DynamicWorld::DefaultGravity);
    }

    TEST(DynamicWorldTest, SetGravityInfinity)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, INFINITY});
        EXPECT_EQ(world.GetGravity(), Guch2D::DynamicWorld::DefaultGravity);
    }

    TEST(DynamicWorldTest, StepAppliesGravityAndMovesBody)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({2.0F, -4.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(2.0F);
        body->SetPosition({1.0F, -1.0F});
        body->SetGravityScale({3.0F, 0.5F});
        body->SetLinearDamping({0.0F, 0.0F});

        world.AddObject(body);

        world.Step();

        EXPECT_EQ(body->GetAcceleration(), Guch2D::Vect(6.0F, -2.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(6.0F, -2.0F));
        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(7.0F, -3.0F));
        EXPECT_EQ(body->GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicWorldTest, StepSkipsIntegrationForZeroMass)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({5.0F, 7.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(0.0F);
        body->SetPosition({1.0F, 2.0F});
        body->SetVelocity({3.0F, 4.0F});
        body->SetAcceleration({5.0F, 6.0F});
        body->SetForce({7.0F, 8.0F});

        world.AddObject(body);

        world.Step();

        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(1.0F, 2.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(3.0F, 4.0F));
        EXPECT_EQ(body->GetAcceleration(), Guch2D::Vect(5.0F, 6.0F));
        EXPECT_EQ(body->GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicWorldTest, StepAppliesLinearDamping)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({0.0F, 0.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(2.0F);
        body->SetVelocity({4.0F, 8.0F});
        body->SetLinearDamping({0.5F, 0.25F});

        world.AddObject(body);

        world.Step();

        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(4.0F, 8.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(2.0F, 6.0F));
        EXPECT_EQ(body->GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body->GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }
}   // namespace
