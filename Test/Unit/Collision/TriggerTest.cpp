#include "Collision/Trigger.hpp"

#include <cstdint>
#include <gtest/gtest.h>
#include <memory>
#include <type_traits>
#include <utility>

#include "Collision/CircleCollider.hpp"
#include "Collision/CollisionWorld.hpp"

namespace
{
    [[nodiscard]] std::shared_ptr<Guch2D::Trigger> MakeTrigger(const Guch2D::Vect& position,
                                                               const float radius)
    {
        auto trigger = std::make_shared<Guch2D::Trigger>();
        trigger->SetPosition(position);
        trigger->SetCollider(std::make_shared<Guch2D::CircleCollider>(radius));
        return trigger;
    }

    TEST(TriggerTest, TypeTraits)
    {
        EXPECT_TRUE((std::is_base_of_v<Guch2D::CollisionBody, Guch2D::Trigger>));
        EXPECT_TRUE(std::is_final_v<Guch2D::Trigger>);
    }

    TEST(TriggerTest, DefaultConstructor)
    {
        const Guch2D::Trigger trigger;
        EXPECT_EQ(trigger.GetPosition(), Guch2D::Vect(0.0F, 0.0F));
        EXPECT_EQ(trigger.GetCollider(), nullptr);
    }

    TEST(TriggerTest, CopyConstructorCopiesState)
    {
        auto collider = std::make_shared<Guch2D::CircleCollider>(1.5F);

        Guch2D::Trigger source;
        source.SetPosition({3.0F, -2.0F});
        source.SetCollider(collider);

        const Guch2D::Trigger copy(source);

        EXPECT_EQ(copy.GetPosition(), Guch2D::Vect(3.0F, -2.0F));
        EXPECT_EQ(copy.GetCollider(), collider);
    }

    TEST(TriggerTest, MoveConstructorMovesState)
    {
        auto collider = std::make_shared<Guch2D::CircleCollider>(2.0F);

        Guch2D::Trigger source;
        source.SetPosition({5.0F, 7.0F});
        source.SetCollider(collider);

        const Guch2D::Trigger moved(std::move(source));

        EXPECT_EQ(moved.GetPosition(), Guch2D::Vect(5.0F, 7.0F));
        EXPECT_EQ(moved.GetCollider(), collider);
    }

    TEST(TriggerTest, AssignmentOperatorsCopyAndMoveState)
    {
        auto colliderA = std::make_shared<Guch2D::CircleCollider>(0.8F);

        Guch2D::Trigger sourceA;
        sourceA.SetPosition({-4.0F, 1.0F});
        sourceA.SetCollider(colliderA);

        Guch2D::Trigger copyAssigned;
        copyAssigned = sourceA;

        EXPECT_EQ(copyAssigned.GetPosition(), Guch2D::Vect(-4.0F, 1.0F));
        EXPECT_EQ(copyAssigned.GetCollider(), colliderA);

        auto colliderB = std::make_shared<Guch2D::CircleCollider>(1.2F);

        Guch2D::Trigger sourceB;
        sourceB.SetPosition({9.0F, -3.0F});
        sourceB.SetCollider(colliderB);

        Guch2D::Trigger moveAssigned;
        moveAssigned = std::move(sourceB);

        EXPECT_EQ(moveAssigned.GetPosition(), Guch2D::Vect(9.0F, -3.0F));
        EXPECT_EQ(moveAssigned.GetCollider(), colliderB);
    }

    TEST(TriggerTest, OverlapCallbacksViaCollisionWorld)
    {
        Guch2D::CollisionWorld world;
        auto triggerA = MakeTrigger({0.0F, 0.0F}, 2.0F);
        auto triggerB = MakeTrigger({3.0F, 0.0F}, 2.0F);

        std::uint8_t beginA = 0;
        std::uint8_t beginB = 0;
        std::uint8_t endA = 0;
        std::uint8_t endB = 0;

        triggerA->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++beginA; });
        triggerB->BindOnBeginOverlap([&](const Guch2D::Collision&) { ++beginB; });
        triggerA->BindOnEndOverlap([&](const Guch2D::Collision&) { ++endA; });
        triggerB->BindOnEndOverlap([&](const Guch2D::Collision&) { ++endB; });

        world.AddObject(triggerA);
        world.AddObject(triggerB);

        world.Step();
        EXPECT_EQ(beginA, 1);
        EXPECT_EQ(beginB, 1);
        EXPECT_EQ(endA, 0);
        EXPECT_EQ(endB, 0);

        // Persistent overlap should not emit duplicate begin callbacks.
        world.Step();
        EXPECT_EQ(beginA, 1);
        EXPECT_EQ(beginB, 1);
        EXPECT_EQ(endA, 0);
        EXPECT_EQ(endB, 0);

        triggerB->SetPosition({10.0F, 0.0F});
        world.Step();
        EXPECT_EQ(endA, 1);
        EXPECT_EQ(endB, 1);
    }
}   // namespace
