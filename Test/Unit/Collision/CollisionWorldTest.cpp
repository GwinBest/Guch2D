#include "Collision/CollisionWorld.hpp"

#include <gtest/gtest.h>

#include "Collision/CircleCollider.hpp"

namespace
{

    class CollisionWorldTest : public Guch2D::CollisionWorld
    {
    public:
        using Guch2D::CollisionWorld::CheckCollisions;
    };

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

    TEST(CollisionWorldTest, StepNoObjects)
    {
        const Guch2D::CollisionWorld world;
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

}   // namespace
