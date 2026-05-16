#include "Dynamics/DynamicWorld.hpp"

#include <array>
#include <gtest/gtest.h>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"
#include "Dynamics/StaticRigidBody.hpp"
#include "Solver/PositionSolver.hpp"
#include "Solver/VelocitySolver.hpp"

namespace
{
    constexpr float RestingContactTolerance = 0.02F;

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
        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(4.0F, -2.0F));
        EXPECT_EQ(body->GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicWorldTest, StepIntegratesBodyWhenSimulatePhysicsTrue)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({1.0F, -2.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetSimulatePhysics(true);
        body->SetMass(2.0F);
        body->SetPosition({3.0F, 4.0F});
        body->SetVelocity({5.0F, 6.0F});
        body->SetAcceleration({0.0F, 0.0F});
        body->SetForce({7.0F, 8.0F});
        body->SetGravityScale({1.0F, 1.0F});
        body->SetLinearDamping({0.0F, 0.0F});

        world.AddObject(body);

        world.Step();

        EXPECT_EQ(body->GetAcceleration(), Guch2D::Vect(4.5F, 2.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(9.5F, 8.0F));
        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(10.25F, 11.0F));
        EXPECT_EQ(body->GetForce(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicWorldTest, StepSkipsBodyWhenSimulatePhysicsFalse)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({1.0F, -2.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetSimulatePhysics(false);
        body->SetMass(2.0F);
        body->SetPosition({3.0F, 4.0F});
        body->SetVelocity({5.0F, 6.0F});
        body->SetAcceleration({1.0F, 2.0F});
        body->SetForce({7.0F, 8.0F});
        body->SetGravityScale({1.0F, 1.0F});
        body->SetLinearDamping({0.0F, 0.0F});

        world.AddObject(body);

        world.Step();

        EXPECT_EQ(body->GetAcceleration(), Guch2D::Vect(1.0F, 2.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(5.0F, 6.0F));
        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(3.0F, 4.0F));
        EXPECT_EQ(body->GetForce(), Guch2D::Vect(7.0F, 8.0F));
    }

    TEST(DynamicWorldTest, StepWakesUnsupportedSleepingBodyUnderGravity)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({1.0F, -2.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(2.0F);
        body->SetPosition({3.0F, 4.0F});
        body->SetLinearDamping({0.0F, 0.0F});
        body->SetAwake(false);

        world.AddObject(body);

        world.Step();

        EXPECT_TRUE(body->IsAwake());
        EXPECT_EQ(body->GetAcceleration(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(3.0F, 4.0F));
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

    TEST(DynamicWorldTest, StepDoesNotSleepBodyWithFastNegativeVelocity)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(1.0F);
        world.SetGravity({0.0F, 0.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(2.0F);
        body->SetVelocity({-4.0F, 0.0F});
        body->SetLinearDamping({0.0F, 0.0F});

        world.AddObject(body);

        world.Step();

        EXPECT_TRUE(body->IsAwake());
        EXPECT_EQ(body->GetPosition(), Guch2D::Vect(-4.0F, 0.0F));
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(-4.0F, 0.0F));
    }

    TEST(DynamicWorldTest, StepDoesNotSleepBodyAtBallisticApex)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, 9.81F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(1.0F);
        body->SetVelocity({0.0F, -9.81F * world.GetTimeStep()});
        body->SetLinearDamping({0.0F, 0.0F});

        world.AddObject(body);

        world.Step();

        EXPECT_TRUE(body->IsAwake());
        EXPECT_NEAR(body->GetVelocity().y, 0.0F, 1.0e-6F);
    }

    TEST(DynamicWorldTest, StepSleepsBodyAfterRemainingUnderVelocityThreshold)
    {
        Guch2D::DynamicWorld world;
        world.SetTimeStep(0.1F);
        world.SetGravity({0.0F, 0.0F});

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>();
        body->SetMass(1.0F);
        body->SetLinearDamping({0.0F, 0.0F});

        world.AddObject(body);

        for (int i = 0; i < 6; ++i)
        {
            world.Step();
        }

        EXPECT_FALSE(body->IsAwake());
        EXPECT_EQ(body->GetVelocity(), Guch2D::Vect(0.0F, 0.0F));
    }

    TEST(DynamicWorldTest, StepKeepsRestingContactWithinPenetrationSlop)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, 9.81F});
        world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());
        world.AddSolver(std::make_shared<Guch2D::PositionSolver>());

        const auto body = std::make_shared<Guch2D::DynamicRigidBody>(Guch2D::Vect {0.0F, -1.0F},
                                                                     1.0F);
        body->SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
        body->SetLinearDamping({0.0F, 0.0F});

        const auto ground = std::make_shared<Guch2D::StaticRigidBody>(Guch2D::Vect {0.0F, 1.0F});
        ground->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {10.0F, 1.0F}));

        world.AddObject(body);
        world.AddObject(ground);

        for (int i = 0; i < 5; ++i)
        {
            world.Step();

            EXPECT_NEAR(body->GetPosition().y, -1.0F, RestingContactTolerance);
            EXPECT_NEAR(body->GetVelocity().y, 0.0F, 1.0e-5F);
        }
    }

    TEST(DynamicWorldTest, StepKeepsStackedBodiesRestingWithoutJitter)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, 9.81F});
        world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());
        world.AddSolver(std::make_shared<Guch2D::PositionSolver>());

        const auto topBody = std::make_shared<Guch2D::DynamicRigidBody>(Guch2D::Vect {0.0F, -3.0F},
                                                                        1.0F);
        topBody->SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
        topBody->SetLinearDamping({0.0F, 0.0F});

        const auto bottomBody = std::make_shared<Guch2D::DynamicRigidBody>(
            Guch2D::Vect {0.0F, -1.0F},
            1.0F);
        bottomBody->SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
        bottomBody->SetLinearDamping({0.0F, 0.0F});

        const auto ground = std::make_shared<Guch2D::StaticRigidBody>(Guch2D::Vect {0.0F, 1.0F});
        ground->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {10.0F, 1.0F}));

        world.AddObject(topBody);
        world.AddObject(bottomBody);
        world.AddObject(ground);

        for (int i = 0; i < 120; ++i)
        {
            world.Step();
        }

        EXPECT_NEAR(topBody->GetPosition().y, -3.0F, RestingContactTolerance);
        EXPECT_NEAR(bottomBody->GetPosition().y, -1.0F, RestingContactTolerance);
        EXPECT_NEAR(topBody->GetVelocity().y, 0.0F, 1.0e-3F);
        EXPECT_NEAR(bottomBody->GetVelocity().y, 0.0F, 1.0e-3F);
    }

    TEST(DynamicWorldTest, StepKeepsTallTowerFromSpringingUp)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, 9.81F});
        world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());
        world.AddSolver(std::make_shared<Guch2D::PositionSolver>());

        constexpr std::size_t BodyCount = 8;
        std::array<std::shared_ptr<Guch2D::DynamicRigidBody>, BodyCount> bodies;

        for (std::size_t index = 0; index < bodies.size(); ++index)
        {
            const float initialY = -1.0F - (2.0F * static_cast<float>(index));
            bodies.at(index) = std::make_shared<Guch2D::DynamicRigidBody>(
                Guch2D::Vect {0.0F, initialY},
                1.0F);
            bodies.at(index)->SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
            bodies.at(index)->SetLinearDamping({0.0F, 0.0F});
            world.AddObject(bodies.at(index));
        }

        const auto ground = std::make_shared<Guch2D::StaticRigidBody>(Guch2D::Vect {0.0F, 1.0F});
        ground->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {10.0F, 1.0F}));
        world.AddObject(ground);

        for (int i = 0; i < 240; ++i)
        {
            world.Step();
        }

        for (std::size_t index = 0; index < bodies.size(); ++index)
        {
            const float initialY = -1.0F - (2.0F * static_cast<float>(index));
            EXPECT_GE(bodies.at(index)->GetPosition().y, initialY - RestingContactTolerance);
            EXPECT_NEAR(bodies.at(index)->GetVelocity().y, 0.0F, 1.0e-3F);
        }
    }

    TEST(DynamicWorldTest, StepSleepsBodyRestingOnStaticColliderUnderGravity)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, 9.81F});
        world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());
        world.AddSolver(std::make_shared<Guch2D::PositionSolver>());

        const auto dynamicBody = std::make_shared<Guch2D::DynamicRigidBody>(
            Guch2D::Vect {0.0F, -1.0F},
            10.0F);
        dynamicBody->SetCollider(std::make_shared<Guch2D::CircleCollider>(0.1F));
        dynamicBody->SetBounciness(0.5F);
        dynamicBody->SetStaticFriction(1.0F);
        dynamicBody->SetDynamicFriction(1.0F);

        const auto floor = std::make_shared<Guch2D::StaticRigidBody>(Guch2D::Vect {0.0F, 0.0F});
        floor->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {5.0F, 0.1F}));
        floor->SetBounciness(1.0F);
        floor->SetStaticFriction(1.0F);
        floor->SetDynamicFriction(1.0F);

        world.AddObject(dynamicBody);
        world.AddObject(floor);

        for (int i = 0; i < 10 * 60; ++i)
        {
            world.Step();
        }

        EXPECT_FALSE(dynamicBody->IsAwake());
    }

    TEST(DynamicWorldTest, StepWakesSleepingStackWhenBottomBodyRemoved)
    {
        Guch2D::DynamicWorld world;
        world.SetGravity({0.0F, 9.81F});
        world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());
        world.AddSolver(std::make_shared<Guch2D::PositionSolver>());

        const auto bottom = std::make_shared<Guch2D::DynamicRigidBody>(Guch2D::Vect {0.0F, -1.0F},
                                                                       1.0F);
        bottom->SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
        bottom->SetLinearDamping({0.0F, 0.0F});
        bottom->SetAwake(false);

        const auto top = std::make_shared<Guch2D::DynamicRigidBody>(Guch2D::Vect {0.0F, -3.0F},
                                                                    1.0F);
        top->SetCollider(std::make_shared<Guch2D::CircleCollider>(1.0F));
        top->SetLinearDamping({0.0F, 0.0F});
        top->SetAwake(false);

        const auto ground = std::make_shared<Guch2D::StaticRigidBody>(Guch2D::Vect {0.0F, 1.0F});
        ground->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {10.0F, 1.0F}));

        world.AddObject(bottom);
        world.AddObject(top);
        world.AddObject(ground);

        world.Step();

        EXPECT_FALSE(bottom->IsAwake());
        EXPECT_FALSE(top->IsAwake());

        world.RemoveObject(bottom);
        world.Step();

        EXPECT_TRUE(top->IsAwake());
    }
}   // namespace
