#include "Solver/PositionSolver.hpp"

#include <gtest/gtest.h>

#include "Dynamics/DynamicRigidBody.hpp"

namespace
{
    TEST(PositionSolverTest, TreatsAsleepDynamicBodyAsStatic)
    {
        const auto asleepBody = std::make_shared<Guch2D::DynamicRigidBody>();
        asleepBody->SetMass(2.0F);
        asleepBody->SetPosition({0.0F, 0.0F});
        asleepBody->SetAwake(false);

        const auto awakeBody = std::make_shared<Guch2D::DynamicRigidBody>();
        awakeBody->SetMass(2.0F);
        awakeBody->SetPosition({0.0F, 0.0F});

        Guch2D::Collision collision;
        collision.BodyA = asleepBody;
        collision.BodyB = awakeBody;
        collision.Points.Normal = {1.0F, 0.0F};
        collision.Points.Depth = 1.0F;
        collision.Points.HasCollision = true;

        Guch2D::PositionSolver solver;
        solver.Solve({collision});

        EXPECT_FALSE(asleepBody->IsAwake());
        EXPECT_EQ(asleepBody->GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(awakeBody->GetPosition(), Guch2D::Vect(-1.0F, 0.0F));
    }
}   // namespace
