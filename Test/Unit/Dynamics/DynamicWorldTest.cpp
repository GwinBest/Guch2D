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
}   // namespace
