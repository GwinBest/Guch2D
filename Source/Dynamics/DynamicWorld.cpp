#include "DynamicWorld.hpp"

#include <algorithm>

#include "Dynamics/DynamicRigidBody.hpp"
#include "Math/Vector.hpp"

namespace
{
    constexpr float SleepVelocityThreshold = 0.01F;
    constexpr float SleepForceThreshold = 0.01F;
    constexpr float SleepTimeThreshold = 0.5F;
    constexpr float SleepSupportNormalThreshold = 0.5F;

    [[nodiscard]] bool
        HasCollisionForBody(const std::vector<Guch2D::Collision>& collisions,
                            const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicRigidBody)
    {
        return std::ranges::any_of(collisions, [&](const Guch2D::Collision& collision) {
            return collision.BodyA.lock() == dynamicRigidBody
                || collision.BodyB.lock() == dynamicRigidBody;
        });
    }

    [[nodiscard]] Guch2D::Vect
        GetGravityForce(const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicRigidBody,
                        const Guch2D::Vect& gravity)
    {
        return dynamicRigidBody->GetMass() * (dynamicRigidBody->GetGravityScale() * gravity);
    }

    [[nodiscard]] bool
        HasSupportAgainstGravity(const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicRigidBody,
                                 const std::vector<Guch2D::CollisionWorld::ObjectType>& objects,
                                 const Guch2D::Vect& gravityForce)
    {
        const float gravityForceLength = Guch2D::VectLength(gravityForce);
        if (gravityForceLength < SleepForceThreshold)
            return true;

        const Guch2D::Vect gravityDirection = gravityForce / gravityForceLength;

        return std::ranges::any_of(objects, [&](const Guch2D::CollisionWorld::ObjectType& object) {
            if (!object || object == dynamicRigidBody)
                return false;

            const auto collisionPoints = Guch2D::CollisionWorld::CheckCollisions(dynamicRigidBody,
                                                                                 object);
            if (!collisionPoints.HasCollision)
                return false;

            const float normalDotGravity = Guch2D::VectDot(collisionPoints.Normal,
                                                           gravityDirection);
            return normalDotGravity <= -SleepSupportNormalThreshold;
        });
    }
}   // namespace

namespace Guch2D
{
    void DynamicWorld::Step()
    {
        ApplyGravity();
        MoveBodies();
        CollisionWorld::Step();
        UpdateSleepStates();
    }

    void DynamicWorld::ApplyGravity() const noexcept
    {
        for (const auto& object : _objects)
        {
            const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object);
            if (dynamicRigidBody && dynamicRigidBody->GetSimulatePhysics()
                && dynamicRigidBody->IsAwake())
            {
                // Apply gravity scaled per-body
                dynamicRigidBody->AddForce(dynamicRigidBody->GetMass()
                                           * (dynamicRigidBody->GetGravityScale() * _gravity));
            }
        }
    }

    void DynamicWorld::UpdateSleepStates() const noexcept
    {
        for (const auto& object : _objects)
        {
            const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object);
            if (!dynamicRigidBody || !dynamicRigidBody->GetSimulatePhysics())
            {
                continue;
            }

            const Vect gravityForce = GetGravityForce(dynamicRigidBody, _gravity);

            if (!dynamicRigidBody->IsAwake())
            {
                if (!HasSupportAgainstGravity(dynamicRigidBody, _objects, gravityForce))
                {
                    dynamicRigidBody->SetAwake(true);
                }

                continue;
            }

            const bool hasGravityForce = VectLength(gravityForce) >= SleepForceThreshold;
            // CollisionWorld::Step() moves the current frame collisions into _previousCollisions
            // at the end of the step. At this point _previousCollisions stores contacts that were
            // solved this frame.
            if (hasGravityForce && !HasCollisionForBody(_previousCollisions, dynamicRigidBody))
            {
                dynamicRigidBody->ResetSleepTime();
                continue;
            }

            if (VectLength(dynamicRigidBody->GetVelocity()) >= SleepVelocityThreshold
                || VectLength(dynamicRigidBody->GetForce()) >= SleepForceThreshold)
            {
                dynamicRigidBody->ResetSleepTime();
                continue;
            }

            dynamicRigidBody->AddSleepTime(_timeStep);
            if (dynamicRigidBody->GetSleepTime() >= SleepTimeThreshold)
            {
                dynamicRigidBody->SetAwake(false);
            }
        }
    }

    void DynamicWorld::MoveBodies() const noexcept
    {
        for (const auto& object : _objects)
        {
            const auto dynamicRigidBody = std::dynamic_pointer_cast<DynamicRigidBody>(object);
            if (dynamicRigidBody && dynamicRigidBody->GetSimulatePhysics()
                && dynamicRigidBody->IsAwake())
            {
                const float mass = dynamicRigidBody->GetMass();
                if (mass == 0.0F)
                {
                    // Skip integration for zero mass
                    dynamicRigidBody->ResetForce();
                    continue;
                }

                dynamicRigidBody->SetAcceleration(dynamicRigidBody->GetForce() / mass);

                constexpr float halfStepFactor = 0.5F;
                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep
                                              * halfStepFactor);

                dynamicRigidBody->UpdatePosition(dynamicRigidBody->GetVelocity() * _timeStep);

                dynamicRigidBody->AddVelocity(dynamicRigidBody->GetAcceleration() * _timeStep
                                              * halfStepFactor);

                ApplyLinearDamping(dynamicRigidBody);

                // Reset force for the next step
                dynamicRigidBody->ResetForce();
            }
        }
    }

    void DynamicWorld::ApplyLinearDamping(
        const std::shared_ptr<DynamicRigidBody>& dynamicRigidBody) const noexcept
    {
        const float dampingFactorX = std::clamp(
            1.0F - (_timeStep * dynamicRigidBody->GetLinearDamping().x),
            0.0F,
            1.0F);
        const float dampingFactorY = std::clamp(
            1.0F - (_timeStep * dynamicRigidBody->GetLinearDamping().y),
            0.0F,
            1.0F);

        dynamicRigidBody->SetVelocity(dynamicRigidBody->GetVelocity()
                                      * Vect(dampingFactorX, dampingFactorY));
    }
}   // namespace Guch2D
