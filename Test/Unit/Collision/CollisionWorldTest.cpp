#include "Collision/CollisionWorld.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"
#include "Solver/PenetrationVectorSolver.hpp"

namespace
{
    class CollisionWorldTest final : public Guch2D::CollisionWorld
    {
    public:
        using Guch2D::CollisionWorld::CheckCollisions;
    };

    [[nodiscard]] std::shared_ptr<Guch2D::CollisionBody>
        MakeAABBBody(const Guch2D::Vect& position,
                     const Guch2D::Vect& extent,
                     const Guch2D::Vect& colliderCenter = {0.0F, 0.0F})
    {
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        body->SetPosition(position);

        const auto collider = std::make_shared<Guch2D::AABBCollider>(extent);
        collider->SetCenterLocal(colliderCenter);
        body->SetCollider(collider);

        return body;
    }

    [[nodiscard]] std::shared_ptr<Guch2D::CollisionBody>
        MakeCircleBody(const Guch2D::Vect& position,
                       const float radius,
                       const Guch2D::Vect& colliderCenter = {0.0F, 0.0F})
    {
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        body->SetPosition(position);

        const auto collider = std::make_shared<Guch2D::CircleCollider>(radius);
        collider->SetCenterLocal(colliderCenter);
        body->SetCollider(collider);

        return body;
    }

    TEST(CollisionWorldTest, DefaultConstructor)
    {
        const Guch2D::CollisionWorld world;
        EXPECT_FLOAT_EQ(world.GetTimeStep(), Guch2D::CollisionWorld::DefaultTimeStep);
    }

    TEST(CollisionWorldTest, AddObject)
    {
        Guch2D::CollisionWorld world;
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        world.AddObject(body);
        EXPECT_EQ(world.GetObjectsCount(), 1U);
    }

    TEST(CollisionWorldTest, AddDuplicateObject)
    {
        Guch2D::CollisionWorld world;
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        world.AddObject(body);
        world.AddObject(body);
        EXPECT_EQ(world.GetObjectsCount(), 1U);
    }

    TEST(CollisonWorldTest, AddObjectNullptr)
    {
        Guch2D::CollisionWorld world;
        const std::shared_ptr<Guch2D::CollisionBody> body = nullptr;
        world.AddObject(body);
        EXPECT_EQ(world.GetObjectsCount(), 0U);
    }

    TEST(CollisionWorldTest, RemoveObject)
    {
        Guch2D::CollisionWorld world;
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        world.AddObject(body);
        world.RemoveObject(body);
        EXPECT_EQ(world.GetObjectsCount(), 0U);
    }

    TEST(CollisionWorldTest, RemoveDuplicateObject)
    {
        Guch2D::CollisionWorld world;
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        world.AddObject(body);
        world.RemoveObject(body);
        world.RemoveObject(body);
        EXPECT_EQ(world.GetObjectsCount(), 0U);
    }

    TEST(CollisionWorldTest, RemoveObjectNullptr)
    {
        Guch2D::CollisionWorld world;
        const std::shared_ptr<Guch2D::CollisionBody> body = nullptr;
        world.RemoveObject(body);
        EXPECT_EQ(world.GetObjectsCount(), 0U);
    }

    TEST(CollisionWorldTest, SetTimeStepPositive)
    {
        Guch2D::CollisionWorld world;
        world.SetTimeStep(0.1F);
        EXPECT_FLOAT_EQ(world.GetTimeStep(), 0.1F);
    }

    TEST(CollisionWorldTest, SetTimeStepZero)
    {
        Guch2D::CollisionWorld world;
        world.SetTimeStep(0.0F);
        EXPECT_FLOAT_EQ(world.GetTimeStep(), Guch2D::CollisionWorld::DefaultTimeStep);
    }

    TEST(CollisionWorldTest, SetTimeStepNegative)
    {
        Guch2D::CollisionWorld world;
        world.SetTimeStep(-0.1F);
        EXPECT_FLOAT_EQ(world.GetTimeStep(), Guch2D::CollisionWorld::DefaultTimeStep);
    }

    TEST(CollisionWorldTest, SetTimeStepInfinite)
    {
        Guch2D::CollisionWorld world;
        world.SetTimeStep(INFINITY);
        EXPECT_FLOAT_EQ(world.GetTimeStep(), Guch2D::CollisionWorld::DefaultTimeStep);
    }

    TEST(CollsionWorldTest, SetTimeStepNaN)
    {
        Guch2D::CollisionWorld world;
        world.SetTimeStep(NAN);
        EXPECT_FLOAT_EQ(world.GetTimeStep(), Guch2D::CollisionWorld::DefaultTimeStep);
    }

    TEST(CollisionWorldTest, GetBroadPhaseTypeDefaultIsSweepAndPrune)
    {
        const Guch2D::CollisionWorld world;
        EXPECT_EQ(world.GetBroadPhaseType(), Guch2D::BroadPhaseType::SweepAndPrune);
    }

    TEST(CollisionWorldTest, SetBroadPhaseTypeSpatialHashing)
    {
        Guch2D::CollisionWorld world;
        world.SetBroadPhaseType(Guch2D::BroadPhaseType::SpatialHashing);
        EXPECT_EQ(world.GetBroadPhaseType(), Guch2D::BroadPhaseType::SpatialHashing);
    }

    TEST(CollisionWorldTest, StepTwoCirclesWithSpatialHashingDetectsCollision)
    {
        Guch2D::CollisionWorld world;
        world.SetBroadPhaseType(Guch2D::BroadPhaseType::SpatialHashing);

        const auto bodyA = MakeCircleBody({0.0F, 0.0F}, 2.0F);
        const auto bodyB = MakeCircleBody({3.0F, 0.0F}, 2.0F);

        bool onBeginOverlapCalledA = false;
        bool onBeginOverlapCalledB = false;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledA = true; });
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledB = true; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_TRUE(onBeginOverlapCalledA);
        EXPECT_TRUE(onBeginOverlapCalledB);
    }

    TEST(CollisionWorldTest, SpatialHashingDeduplicatesCollisionPairAcrossNeighborCells)
    {
        Guch2D::CollisionWorld world;
        world.SetBroadPhaseType(Guch2D::BroadPhaseType::SpatialHashing);

        const auto bodyA = MakeCircleBody({3.9F, 0.0F}, 1.0F);
        const auto bodyB = MakeCircleBody({4.1F, 0.0F}, 1.0F);

        std::uint8_t onBeginOverlapCalledACount = 0;
        std::uint8_t onBeginOverlapCalledBCount = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledACount; });
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledBCount; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
    }

    TEST(CollisionWorldTest, StepNoObjects)
    {
        Guch2D::CollisionWorld world;
        world.Step();
        SUCCEED();
    }

    TEST(CollisionWorldTest, StepOneCircleNoCollision)
    {
        Guch2D::CollisionWorld world;
        const auto body = std::make_shared<Guch2D::CollisionBody>();
        body->SetCollider(std::make_shared<Guch2D::CircleCollider>());

        bool onBeginOverlapCalled = false;
        bool onEndOverlapCalled = false;
        body->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalled = true; });
        body->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalled = true; });

        world.AddObject(body);
        world.Step();

        EXPECT_FALSE(onBeginOverlapCalled);
        EXPECT_FALSE(onEndOverlapCalled);
    }

    TEST(CollisionWorldTest, StepTwoCirclesNoCollision)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(0.5F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({5.0F, 5.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(0.5F);

        bool onBeginOverlapCalledA = false;
        bool onEndOverlapCalledA = false;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledA = true; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalledA = true; });

        bool onBeginOverlapCalledB = false;
        bool onEndOverlapCalledB = false;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledB = true; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalledB = true; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_FALSE(onBeginOverlapCalledA);
        EXPECT_FALSE(onEndOverlapCalledA);
        EXPECT_FALSE(onBeginOverlapCalledB);
        EXPECT_FALSE(onEndOverlapCalledB);
    }

    TEST(CollisionWorldTest, StepTwoCirclesOverlapsTwoSteps)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(5.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({5.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(5.0F);

        std::uint8_t onBeginOverlapCalledACount = 0;
        std::uint8_t onEndOverlapCalledACount = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledACount; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { ++onEndOverlapCalledACount; });

        std::uint8_t onBeginOverlapCalledBCount = 0;
        std::uint8_t onEndOverlapCalledBCount = 0;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledBCount; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { ++onEndOverlapCalledBCount; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onEndOverlapCalledACount, 0);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
        EXPECT_EQ(onEndOverlapCalledBCount, 0);

        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onEndOverlapCalledACount, 0);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
        EXPECT_EQ(onEndOverlapCalledBCount, 0);
    }

    TEST(CollisionWorld, StepTwoCirclesWithCollision)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({4.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        bool onBeginOverlapCalledA = false;
        bool onEndOverlapCalledA = false;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledA = true; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalledA = true; });

        bool onBeginOverlapCalledB = false;
        bool onEndOverlapCalledB = false;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledB = true; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalledB = true; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_TRUE(onBeginOverlapCalledA);
        EXPECT_FALSE(onEndOverlapCalledA);
        EXPECT_TRUE(onBeginOverlapCalledB);
        EXPECT_FALSE(onEndOverlapCalledB);
    }

    TEST(CollisionWorld, StepTwoCirclesCollisionBeginEnd)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({4.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        bool onBeginOverlapCalledA = false;
        bool onEndOverlapCalledA = false;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledA = true; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalledA = true; });

        bool onBeginOverlapCalledB = false;
        bool onEndOverlapCalledB = false;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { onBeginOverlapCalledB = true; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { onEndOverlapCalledB = true; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();   // Begin overlap

        EXPECT_TRUE(onBeginOverlapCalledA);
        EXPECT_FALSE(onEndOverlapCalledA);
        EXPECT_TRUE(onBeginOverlapCalledB);
        EXPECT_FALSE(onEndOverlapCalledB);

        // Move bodyB away to end overlap
        bodyB->SetPosition({4.1F, 0.0F});
        world.Step();   // End overlap

        EXPECT_TRUE(onBeginOverlapCalledA);
        EXPECT_TRUE(onEndOverlapCalledA);
        EXPECT_TRUE(onBeginOverlapCalledB);
        EXPECT_TRUE(onEndOverlapCalledB);
    }

    TEST(CollisionWorld, StepThreeCirclesMultipleCollisions)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({4.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        const auto bodyC = std::make_shared<Guch2D::CollisionBody>();
        bodyC->SetPosition({2.0F, 2.0F});
        bodyC->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyC->GetCollider())->SetRadius(2.0F);

        int beginOverlapCountA = 0;
        int endOverlapCountA = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountA++; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountA++; });

        int beginOverlapCountB = 0;
        int endOverlapCountB = 0;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountB++; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountB++; });

        int beginOverlapCountC = 0;
        int endOverlapCountC = 0;
        bodyC->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountC++; });
        bodyC->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountC++; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.AddObject(bodyC);

        world.Step();

        EXPECT_EQ(beginOverlapCountA, 2);
        EXPECT_EQ(endOverlapCountA, 0);
        EXPECT_EQ(beginOverlapCountB, 2);
        EXPECT_EQ(endOverlapCountB, 0);
        EXPECT_EQ(beginOverlapCountC, 2);
        EXPECT_EQ(endOverlapCountC, 0);
    }

    TEST(CollisionWorld, StepThreeCirclesMultipleCollisionsEnd)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({4.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        const auto bodyC = std::make_shared<Guch2D::CollisionBody>();
        bodyC->SetPosition({2.0F, 2.0F});
        bodyC->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyC->GetCollider())->SetRadius(2.0F);

        int beginOverlapCountA = 0;
        int endOverlapCountA = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountA++; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountA++; });

        int beginOverlapCountB = 0;
        int endOverlapCountB = 0;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountB++; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountB++; });

        int beginOverlapCountC = 0;
        int endOverlapCountC = 0;
        bodyC->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountC++; });
        bodyC->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountC++; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.AddObject(bodyC);

        world.Step();

        EXPECT_EQ(beginOverlapCountA, 2);
        EXPECT_EQ(endOverlapCountA, 0);
        EXPECT_EQ(beginOverlapCountB, 2);
        EXPECT_EQ(endOverlapCountB, 0);
        EXPECT_EQ(beginOverlapCountC, 2);
        EXPECT_EQ(endOverlapCountC, 0);

        // Move bodies apart to end all overlaps
        bodyA->SetPosition({-10.0F, -10.0F});
        bodyB->SetPosition({10.0F, 10.0F});
        bodyC->SetPosition({10.0F, -10.0F});

        world.Step();

        EXPECT_EQ(beginOverlapCountA, 2);
        EXPECT_EQ(endOverlapCountA, 2);
        EXPECT_EQ(beginOverlapCountB, 2);
        EXPECT_EQ(endOverlapCountB, 2);
        EXPECT_EQ(beginOverlapCountC, 2);
        EXPECT_EQ(endOverlapCountC, 2);
    }

    TEST(CollisionWorld, StepThreeCirclesWithOneCollision)
    {
        Guch2D::CollisionWorld world;

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({4.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        const auto bodyC = std::make_shared<Guch2D::CollisionBody>();
        bodyC->SetPosition({50.0F, 5.0F});
        bodyC->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyC->GetCollider())->SetRadius(2.0F);

        int beginOverlapCountA = 0;
        int endOverlapCountA = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountA++; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountA++; });

        int beginOverlapCountB = 0;
        int endOverlapCountB = 0;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountB++; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountB++; });

        int beginOverlapCountC = 0;
        int endOverlapCountC = 0;
        bodyC->BindOnBeginOverlap([&](const Guch2D::Collision&) { beginOverlapCountC++; });
        bodyC->BindOnEndOverlap([&](const Guch2D::Collision&) { endOverlapCountC++; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.AddObject(bodyC);

        world.Step();

        EXPECT_EQ(beginOverlapCountA, 1);
        EXPECT_EQ(endOverlapCountA, 0);
        EXPECT_EQ(beginOverlapCountB, 1);
        EXPECT_EQ(endOverlapCountB, 0);
        EXPECT_EQ(beginOverlapCountC, 0);
        EXPECT_EQ(endOverlapCountC, 0);

        // Move bodies apart to end all overlaps
        bodyA->SetPosition({-10.0F, -10.0F});
        bodyB->SetPosition({10.0F, 10.0F});
        bodyC->SetPosition({10.0F, -10.0F});

        world.Step();

        EXPECT_EQ(beginOverlapCountA, 1);
        EXPECT_EQ(endOverlapCountA, 1);
        EXPECT_EQ(beginOverlapCountB, 1);
        EXPECT_EQ(endOverlapCountB, 1);
        EXPECT_EQ(beginOverlapCountC, 0);
        EXPECT_EQ(endOverlapCountC, 0);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsAABBOverlapX)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {2.0F, 1.0F}));

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({3.0F, 0.5F});
        bodyB->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {2.0F, 1.0F}));

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, -0.5F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 1.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsAABBOverlapY)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {1.0F, 2.0F}));

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({0.25F, 3.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect {1.0F, 2.0F}));

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, -0.75F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 1.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleNoCollisionOutsideFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 1.0F});
        const auto bodyB = MakeCircleBody({4.1F, 0.0F}, 2.0F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleNoCollisionOutsideCorner)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({4.0F, 4.0F}, 2.8F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleTouchingFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 1.0F});
        const auto bodyB = MakeCircleBody({4.0F, 0.0F}, 2.0F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 0.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCirclePenetratingFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 1.0F});
        const auto bodyB = MakeCircleBody({3.0F, 0.0F}, 2.0F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 0.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleTouchingCorner)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({4.0F, 4.0F}, std::sqrt(8.0F));

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_NEAR(collisionPoints.Normal.x, -0.70710677F, 1.0e-6F);
        EXPECT_NEAR(collisionPoints.Normal.y, -0.70710677F, 1.0e-6F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 2.0F);
        EXPECT_NEAR(collisionPoints.ContactPoints.back().x, 2.0F, 1.0e-5F);
        EXPECT_NEAR(collisionPoints.ContactPoints.back().y, 2.0F, 1.0e-5F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCirclePenetratingCorner)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({4.0F, 4.0F}, 3.0F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_NEAR(collisionPoints.Normal.x, -0.70710677F, 1.0e-6F);
        EXPECT_NEAR(collisionPoints.Normal.y, -0.70710677F, 1.0e-6F);
        EXPECT_NEAR(collisionPoints.Depth, 0.17157292F, 1.0e-6F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 2.0F);
        EXPECT_NEAR(collisionPoints.ContactPoints.back().x, 1.87867963F, 1.0e-6F);
        EXPECT_NEAR(collisionPoints.ContactPoints.back().y, 1.87867963F, 1.0e-6F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleInsideChoosesLeftFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({-1.8F, 0.0F}, 0.5F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 0.7F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, -2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, -1.3F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 0.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleInsideChoosesRightFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({1.8F, 0.0F}, 0.5F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 0.7F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 1.3F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 0.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleInsideChoosesBottomFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({0.0F, -1.75F}, 0.5F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 0.75F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, -2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, -1.25F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleInsideChoosesTopFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({0.0F, 1.75F}, 0.5F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 0.75F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 1.25F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleInsideEqualDistanceChoosesLeftFace)
    {
        const auto bodyA = MakeAABBBody({0.0F, 0.0F}, {2.0F, 2.0F});
        const auto bodyB = MakeCircleBody({0.0F, 0.0F}, 0.5F);

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, 1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 2.5F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, -2.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 0.5F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 0.0F);
    }

    TEST(CollisionWorld, CheckCollisionsAABBVsCircleUsesColliderLocalCenter)
    {
        const auto bodyA = MakeAABBBody({10.0F, 10.0F}, {2.0F, 2.0F}, {1.0F, -1.0F});
        const auto bodyB = MakeCircleBody({12.0F, 9.0F}, 1.5F, {1.0F, 0.0F});

        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_TRUE(collisionPoints.HasCollision);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.x, -1.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Normal.y, 0.0F);
        EXPECT_FLOAT_EQ(collisionPoints.Depth, 1.5F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().x, 13.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.front().y, 9.0F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().x, 11.5F);
        EXPECT_FLOAT_EQ(collisionPoints.ContactPoints.back().y, 9.0F);
    }

    TEST(CollisionWorld, CheckCollisionsCircleVsAABBReturnsSwappedNormalAndContacts)
    {
        const auto aabbBody = MakeAABBBody({0.0F, 0.0F}, {2.0F, 1.0F});
        const auto circleBody = MakeCircleBody({3.0F, 0.0F}, 2.0F);

        const auto aabbVsCircle = CollisionWorldTest::CheckCollisions(aabbBody, circleBody);
        const auto circleVsAabb = CollisionWorldTest::CheckCollisions(circleBody, aabbBody);

        EXPECT_TRUE(aabbVsCircle.HasCollision);
        EXPECT_TRUE(circleVsAabb.HasCollision);
        EXPECT_FLOAT_EQ(aabbVsCircle.Depth, circleVsAabb.Depth);
        EXPECT_FLOAT_EQ(aabbVsCircle.Normal.x, -circleVsAabb.Normal.x);
        EXPECT_FLOAT_EQ(aabbVsCircle.Normal.y, -circleVsAabb.Normal.y);
        EXPECT_FLOAT_EQ(aabbVsCircle.ContactPoints.front().x, circleVsAabb.ContactPoints.back().x);
        EXPECT_FLOAT_EQ(aabbVsCircle.ContactPoints.front().y, circleVsAabb.ContactPoints.back().y);
        EXPECT_FLOAT_EQ(aabbVsCircle.ContactPoints.back().x, circleVsAabb.ContactPoints.front().x);
        EXPECT_FLOAT_EQ(aabbVsCircle.ContactPoints.back().y, circleVsAabb.ContactPoints.front().y);
    }

    TEST(CollisionWorld, CheckCollisionsNullBodyA)
    {
        const auto bodyA = std::shared_ptr<Guch2D::CollisionBody>(nullptr);
        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsNullBodyB)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        const auto bodyB = std::shared_ptr<Guch2D::CollisionBody>(nullptr);
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsBothNullBodies)
    {
        const auto bodyA = std::shared_ptr<Guch2D::CollisionBody>(nullptr);
        const auto bodyB = std::shared_ptr<Guch2D::CollisionBody>(nullptr);
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsNoColliderA)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsNoColliderB)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsNoColliders)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorld, CheckCollisionsNoCollisionFunc)
    {
        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        bodyA->GetCollider()->SetColliderType(Guch2D::ColliderType::Count);
        bodyB->GetCollider()->SetColliderType(Guch2D::ColliderType::Count);
        const auto collisionPoints = CollisionWorldTest::CheckCollisions(bodyA, bodyB);
        EXPECT_FALSE(collisionPoints.HasCollision);
    }

    TEST(CollisionWorldTest, AddSolver)
    {
        Guch2D::CollisionWorld world;
        const auto solver = std::make_shared<Guch2D::PenetrationVectorSolver>();
        world.AddSolver(solver);
        EXPECT_EQ(world.GetSolversCount(), 1U);
    }

    TEST(CollisionWorldTest, AddDuplicateSolver)
    {
        Guch2D::CollisionWorld world;
        const auto solver = std::make_shared<Guch2D::PenetrationVectorSolver>();
        world.AddSolver(solver);
        world.AddSolver(solver);
        EXPECT_EQ(world.GetSolversCount(), 1U);
    }

    TEST(CollisonWorldTest, AddSolverNullptr)
    {
        Guch2D::CollisionWorld world;
        const std::shared_ptr<Guch2D::PenetrationVectorSolver> solver = nullptr;
        world.AddSolver(solver);
        EXPECT_EQ(world.GetSolversCount(), 0U);
    }

    TEST(CollisionWorldTest, RemoveSolver)
    {
        Guch2D::CollisionWorld world;
        const auto solver = std::make_shared<Guch2D::PenetrationVectorSolver>();
        world.AddSolver(solver);
        world.RemoveSolver(solver);
        EXPECT_EQ(world.GetSolversCount(), 0U);
    }

    TEST(CollisionWorldTest, RemoveDuplicateSolver)
    {
        Guch2D::CollisionWorld world;
        const auto solver = std::make_shared<Guch2D::PenetrationVectorSolver>();
        world.AddSolver(solver);
        world.RemoveSolver(solver);
        world.RemoveSolver(solver);
        EXPECT_EQ(world.GetSolversCount(), 0U);
    }

    TEST(CollisionWorldTest, RemoveSolverNullptr)
    {
        Guch2D::CollisionWorld world;
        const std::shared_ptr<Guch2D::PenetrationVectorSolver> solver = nullptr;
        world.AddSolver(solver);
        world.RemoveSolver(solver);
        EXPECT_EQ(world.GetSolversCount(), 0U);
    }

    TEST(CollisionWorldTest, StepTwoCirclesWithAddedPenetrationSolverResolvesPositions)
    {
        Guch2D::CollisionWorld world;
        world.AddSolver(std::make_shared<Guch2D::PenetrationVectorSolver>());

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({0.0F, 0.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({4.0F, 0.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        std::uint8_t onBeginOverlapCalledACount = 0;
        std::uint8_t onEndOverlapCalledACount = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledACount; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { ++onEndOverlapCalledACount; });

        std::uint8_t onBeginOverlapCalledBCount = 0;
        std::uint8_t onEndOverlapCalledBCount = 0;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledBCount; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { ++onEndOverlapCalledBCount; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onEndOverlapCalledACount, 0);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
        EXPECT_EQ(onEndOverlapCalledBCount, 0);

        EXPECT_FLOAT_EQ(bodyA->GetPosition().x, 0.0F);
        EXPECT_FLOAT_EQ(bodyA->GetPosition().y, 0.0F);
        EXPECT_FLOAT_EQ(bodyB->GetPosition().x, 4.0F);
        EXPECT_FLOAT_EQ(bodyB->GetPosition().y, 0.0F);

        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onEndOverlapCalledACount, 0);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
        EXPECT_EQ(onEndOverlapCalledBCount, 0);

        EXPECT_FLOAT_EQ(bodyA->GetPosition().x, 0.0F);
        EXPECT_FLOAT_EQ(bodyA->GetPosition().y, 0.0F);
        EXPECT_FLOAT_EQ(bodyB->GetPosition().x, 4.0F);
        EXPECT_FLOAT_EQ(bodyB->GetPosition().y, 0.0F);
    }

    TEST(CollisionWorldTest, StepTwoCirclesSamePositionWithAddedPenetrationSolverResolvesPositions)
    {
        Guch2D::CollisionWorld world;
        world.AddSolver(std::make_shared<Guch2D::PenetrationVectorSolver>());

        const auto bodyA = std::make_shared<Guch2D::CollisionBody>();
        bodyA->SetPosition({1.0F, 1.0F});
        bodyA->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyA->GetCollider())->SetRadius(2.0F);

        const auto bodyB = std::make_shared<Guch2D::CollisionBody>();
        bodyB->SetPosition({1.0F, 1.0F});
        bodyB->SetCollider(std::make_shared<Guch2D::CircleCollider>());
        std::dynamic_pointer_cast<Guch2D::CircleCollider>(bodyB->GetCollider())->SetRadius(2.0F);

        std::uint8_t onBeginOverlapCalledACount = 0;
        std::uint8_t onEndOverlapCalledACount = 0;
        bodyA->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledACount; });
        bodyA->BindOnEndOverlap([&](const Guch2D::Collision&) { ++onEndOverlapCalledACount; });

        std::uint8_t onBeginOverlapCalledBCount = 0;
        std::uint8_t onEndOverlapCalledBCount = 0;
        bodyB->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++onBeginOverlapCalledBCount; });
        bodyB->BindOnEndOverlap([&](const Guch2D::Collision&) { ++onEndOverlapCalledBCount; });

        world.AddObject(bodyA);
        world.AddObject(bodyB);
        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onEndOverlapCalledACount, 0);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
        EXPECT_EQ(onEndOverlapCalledBCount, 0);

        EXPECT_TRUE((bodyA->GetPosition().x == -1.0F && bodyB->GetPosition().x == 3.0F)
                    || (bodyA->GetPosition().x == 3.0F && bodyB->GetPosition().x == -1.0F));
        EXPECT_FLOAT_EQ(bodyA->GetPosition().y, 1.0F);
        EXPECT_FLOAT_EQ(bodyB->GetPosition().y, 1.0F);

        world.Step();

        EXPECT_EQ(onBeginOverlapCalledACount, 1);
        EXPECT_EQ(onEndOverlapCalledACount, 0);
        EXPECT_EQ(onBeginOverlapCalledBCount, 1);
        EXPECT_EQ(onEndOverlapCalledBCount, 0);

        EXPECT_TRUE((bodyA->GetPosition().x == -1.0F && bodyB->GetPosition().x == 3.0F)
                    || (bodyA->GetPosition().x == 3.0F && bodyB->GetPosition().x == -1.0F));
        EXPECT_FLOAT_EQ(bodyA->GetPosition().y, 1.0F);
        EXPECT_FLOAT_EQ(bodyB->GetPosition().y, 1.0F);
    }
}   // namespace
