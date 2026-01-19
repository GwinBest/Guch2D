#include "Dynamics/StaticRigidBody.hpp"

#include <gtest/gtest.h>

namespace
{
    TEST(StaticRigidBodyTest, DefaultConstructor)
    {
        const Guch2D::StaticRigidBody body;
        EXPECT_EQ(body.GetMass(), std::numeric_limits<float>::infinity());
        EXPECT_EQ(body.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(body.GetCollider(), nullptr);
    }

    TEST(StaticRigidBodyTest, PositionConstructor)
    {
        constexpr Guch2D::Vect position(7.0F, -2.0F);
        const Guch2D::StaticRigidBody body(position);
        EXPECT_EQ(body.GetMass(), std::numeric_limits<float>::infinity());
        EXPECT_EQ(body.GetPosition(), position);
        EXPECT_EQ(body.GetCollider(), nullptr);
    }
}   // namespace
