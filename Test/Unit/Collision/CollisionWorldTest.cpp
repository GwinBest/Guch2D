#include "Collision/CollisionWorld.hpp"

#include <gtest/gtest.h>

namespace
{
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
}   // namespace
