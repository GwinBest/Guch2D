#include "VelocitySolver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "Collision/CollisionBody.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/RigidBody.hpp"

namespace
{
    constexpr float RestitutionVelocityThreshold = 1.0F;
    constexpr float WarmStartSeparatingVelocityThreshold = -0.01F;

    [[nodiscard]] Guch2D::Vect GetTangent(const Guch2D::Vect& normal) noexcept
    {
        return {-normal.y, normal.x};
    }

    [[nodiscard]] float GetInverseMass(const std::shared_ptr<Guch2D::RigidBody>& rigidBody) noexcept
    {
        if (!rigidBody)
            return 0.0F;

        const float mass = rigidBody->GetMass();
        return mass == 0.0F ? 0.0F : 1.0F / mass;
    }

    [[nodiscard]] Guch2D::ContactPoint& GetSolverContact(Guch2D::CollisionPoints& points) noexcept
    {
        // The current velocity solver treats each manifold as one linear constraint.
        return points.ContactPoints.front();
    }

    [[nodiscard]] const Guch2D::ContactPoint&
        GetSolverContact(const Guch2D::CollisionPoints& points) noexcept
    {
        return points.ContactPoints.front();
    }

    [[nodiscard]] float GetVelocityAlongNormal(const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                                               const std::shared_ptr<Guch2D::CollisionBody>& bodyB,
                                               const Guch2D::Vect& normal)
    {
        const auto dynamicBodyA = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyA);
        const auto dynamicBodyB = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyB);

        const Guch2D::Vect velocityA = dynamicBodyA ? dynamicBodyA->GetVelocity()
                                                    : Guch2D::Vect {0.0F, 0.0F};
        const Guch2D::Vect velocityB = dynamicBodyB ? dynamicBodyB->GetVelocity()
                                                    : Guch2D::Vect {0.0F, 0.0F};

        return Guch2D::VectDot(velocityB - velocityA, normal);
    }

    void ApplyImpulse(
        const Guch2D::CollisionPoints& points,
        const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicBodyA,
        const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicBodyB,
        const float invMassA,                  // NOLINT(bugprone-easily-swappable-parameters)
        const float invMassB,                  // NOLINT(bugprone-easily-swappable-parameters)
        const float normalImpulseMagnitude,    // NOLINT(bugprone-easily-swappable-parameters)
        const float tangentImpulseMagnitude)   // NOLINT(bugprone-easily-swappable-parameters)
    {
        const Guch2D::Vect impulse = (points.Normal * normalImpulseMagnitude)
                                   + (GetTangent(points.Normal) * tangentImpulseMagnitude);

        if (dynamicBodyA)
        {
            dynamicBodyA->AddVelocity(impulse * invMassA);
        }

        if (dynamicBodyB)
        {
            dynamicBodyB->AddVelocity(-(impulse * invMassB));
        }
    }

    [[nodiscard]] bool HasSameBodyPair(const Guch2D::Collision& lhs, const Guch2D::Collision& rhs)
    {
        const auto lhsBodyA = lhs.BodyA.lock();
        const auto lhsBodyB = lhs.BodyB.lock();
        const auto rhsBodyA = rhs.BodyA.lock();
        const auto rhsBodyB = rhs.BodyB.lock();

        if (!lhsBodyA || !lhsBodyB || !rhsBodyA || !rhsBodyB)
            return false;

        return (lhsBodyA == rhsBodyA && lhsBodyB == rhsBodyB)
            || (lhsBodyA == rhsBodyB && lhsBodyB == rhsBodyA);
    }

    [[nodiscard]] bool HasReversedBodyPair(const Guch2D::Collision& lhs,
                                           const Guch2D::Collision& rhs)
    {
        const auto lhsBodyA = lhs.BodyA.lock();
        const auto lhsBodyB = lhs.BodyB.lock();
        const auto rhsBodyA = rhs.BodyA.lock();
        const auto rhsBodyB = rhs.BodyB.lock();

        if (!lhsBodyA || !lhsBodyB || !rhsBodyA || !rhsBodyB)
            return false;

        return lhsBodyA == rhsBodyB && lhsBodyB == rhsBodyA;
    }

    [[nodiscard]] bool CanWarmStartFrom(const Guch2D::Collision& collision,
                                        const Guch2D::Collision& previousCollision)
    {
        if (!HasSameBodyPair(collision, previousCollision))
            return false;

        constexpr float minNormalDot = 0.5F;
        const Guch2D::Vect previousNormal = HasReversedBodyPair(collision, previousCollision)
                                              ? -previousCollision.Points.Normal
                                              : previousCollision.Points.Normal;

        return Guch2D::VectDot(collision.Points.Normal, previousNormal) >= minNormalDot;
    }

    void CopyPreviousImpulses(Guch2D::Collision& collision,
                              const std::vector<Guch2D::Collision>& previousCollisions)
    {
        const auto previousCollision = std::ranges::find_if(
            previousCollisions,
            [&](const Guch2D::Collision& candidate) {
                return CanWarmStartFrom(collision, candidate);
            });

        if (previousCollision == previousCollisions.end())
            return;

        auto& contact = GetSolverContact(collision.Points);
        const auto& previousContact = GetSolverContact(previousCollision->Points);

        contact.AccumulatedNormalImpulse = previousContact.AccumulatedNormalImpulse;
        contact.AccumulatedTangentImpulse = previousContact.AccumulatedTangentImpulse;
    }

    [[nodiscard]] float CalculateRestitutionVelocityBias(const Guch2D::Collision& collision)
    {
        const auto bodyA = collision.BodyA.lock();
        const auto bodyB = collision.BodyB.lock();

        if (!bodyA || !bodyB)
            return 0.0F;

        const auto rigidBodyA = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyA);
        const auto rigidBodyB = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyB);

        if (!rigidBodyA || !rigidBodyB)
            return 0.0F;

        const float velocityAlongNormal = GetVelocityAlongNormal(bodyA,
                                                                 bodyB,
                                                                 collision.Points.Normal);

        if (velocityAlongNormal <= RestitutionVelocityThreshold)
            return 0.0F;

        const float bounciness = std::min(rigidBodyA->GetBounciness(), rigidBodyB->GetBounciness());
        return bounciness * velocityAlongNormal;
    }

    [[nodiscard]] std::vector<float>
        CalculateRestitutionVelocityBiases(const std::vector<Guch2D::Collision>& collisions)
    {
        std::vector<float> restitutionVelocityBiases;
        restitutionVelocityBiases.reserve(collisions.size());

        for (const auto& collision : collisions)
        {
            restitutionVelocityBiases.push_back(CalculateRestitutionVelocityBias(collision));
        }

        return restitutionVelocityBiases;
    }

    void WarmStartCollision(const Guch2D::CollisionPoints& points,
                            const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicBodyA,
                            const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicBodyB,
                            const float invMassA,   // NOLINT(bugprone-easily-swappable-parameters)
                            const float invMassB)   // NOLINT(bugprone-easily-swappable-parameters)
    {
        const auto& contact = GetSolverContact(points);

        if (contact.AccumulatedNormalImpulse == 0.0F && contact.AccumulatedTangentImpulse == 0.0F)
        {
            return;
        }

        ApplyImpulse(points,
                     dynamicBodyA,
                     dynamicBodyB,
                     invMassA,
                     invMassB,
                     contact.AccumulatedNormalImpulse,
                     contact.AccumulatedTangentImpulse);
    }

    void CalculateTangentialImpulse(
        Guch2D::CollisionPoints& points,
        const std::shared_ptr<Guch2D::RigidBody>& rigidBodyA,
        const std::shared_ptr<Guch2D::RigidBody>& rigidBodyB,
        const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicBodyA,
        const std::shared_ptr<Guch2D::DynamicRigidBody>& dynamicBodyB,
        const Guch2D::Vect& deltaVelocity,
        const float invMassA,   // NOLINT(bugprone-easily-swappable-parameters)
        const float invMassB)   // NOLINT(bugprone-easily-swappable-parameters)
    {
        auto& contact = GetSolverContact(points);
        const Guch2D::Vect tangent = GetTangent(points.Normal);

        float deltaTangentImpulse = Guch2D::VectDot(deltaVelocity, tangent);
        deltaTangentImpulse /= invMassA + invMassB;

        const float oldTangentImpulse = contact.AccumulatedTangentImpulse;
        const float targetTangentImpulse = oldTangentImpulse + deltaTangentImpulse;
        const float muStatic = std::sqrt(rigidBodyA->GetStaticFriction()
                                         * rigidBodyB->GetStaticFriction());
        const float muDynamic = std::sqrt(rigidBodyA->GetDynamicFriction()
                                          * rigidBodyB->GetDynamicFriction());
        const float staticLimit = contact.AccumulatedNormalImpulse * muStatic;
        const float dynamicLimit = contact.AccumulatedNormalImpulse * muDynamic;
        const float frictionLimit = std::abs(targetTangentImpulse) <= staticLimit ? staticLimit
                                                                                  : dynamicLimit;

        contact.AccumulatedTangentImpulse = std::clamp(targetTangentImpulse,
                                                       -frictionLimit,
                                                       frictionLimit);

        const float actualTangentImpulse = contact.AccumulatedTangentImpulse - oldTangentImpulse;

        ApplyImpulse(points,
                     dynamicBodyA,
                     dynamicBodyB,
                     invMassA,
                     invMassB,
                     0.0F,
                     actualTangentImpulse);
    }

    void PrepareWarmStarting(std::vector<Guch2D::Collision>& collisions,
                             const std::vector<Guch2D::Collision>& previousCollisions)
    {
        for (auto& collision : collisions)
        {
            CopyPreviousImpulses(collision, previousCollisions);

            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();

            if (!bodyA || !bodyB)
                continue;

            const auto rigidBodyA = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyA);
            const auto rigidBodyB = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyB);

            if (!rigidBodyA || !rigidBodyB)
                continue;

            const auto dynamicBodyA = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyA);
            const auto dynamicBodyB = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyB);

            const float invMassA = GetInverseMass(rigidBodyA);
            const float invMassB = GetInverseMass(rigidBodyB);

            if (invMassA + invMassB == 0.0F)
                continue;

            auto& contact = GetSolverContact(collision.Points);
            const float velocityAlongNormal = GetVelocityAlongNormal(bodyA,
                                                                     bodyB,
                                                                     collision.Points.Normal);

            if (velocityAlongNormal < WarmStartSeparatingVelocityThreshold)
            {
                contact.AccumulatedNormalImpulse = 0.0F;
                contact.AccumulatedTangentImpulse = 0.0F;
                continue;
            }

            WarmStartCollision(collision.Points, dynamicBodyA, dynamicBodyB, invMassA, invMassB);
        }
    }

    void RemoveRestitutionFromCachedImpulses(std::vector<Guch2D::Collision>& collisions,
                                             const std::vector<float>& restitutionVelocityBiases)
    {
        for (size_t index = 0; index < collisions.size(); ++index)
        {
            if (index >= restitutionVelocityBiases.size()
                || restitutionVelocityBiases.at(index) == 0.0F)
            {
                continue;
            }

            auto& collision = collisions.at(index);
            const auto bodyA = collision.BodyA.lock();
            const auto bodyB = collision.BodyB.lock();

            if (!bodyA || !bodyB)
                continue;

            const auto rigidBodyA = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyA);
            const auto rigidBodyB = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyB);

            if (!rigidBodyA || !rigidBodyB)
                continue;

            const float invMassSum = GetInverseMass(rigidBodyA) + GetInverseMass(rigidBodyB);

            if (invMassSum == 0.0F)
                continue;

            auto& contact = GetSolverContact(collision.Points);
            const float restitutionImpulse = restitutionVelocityBiases.at(index) / invMassSum;

            contact.AccumulatedNormalImpulse = std::max(
                contact.AccumulatedNormalImpulse - restitutionImpulse,
                0.0F);

            const float muStatic = std::sqrt(rigidBodyA->GetStaticFriction()
                                             * rigidBodyB->GetStaticFriction());
            const float muDynamic = std::sqrt(rigidBodyA->GetDynamicFriction()
                                              * rigidBodyB->GetDynamicFriction());
            const float tangentLimit = contact.AccumulatedNormalImpulse
                                     * std::max(muStatic, muDynamic);

            contact.AccumulatedTangentImpulse = std::clamp(contact.AccumulatedTangentImpulse,
                                                           -tangentLimit,
                                                           tangentLimit);
        }
    }

    void SingleVelocityIteration(std::vector<Guch2D::Collision>& collisions,
                                 const std::vector<float>& restitutionVelocityBiases)
    {
        for (size_t index = 0; index < collisions.size(); ++index)
        {
            auto& [BodyA, BodyB, Points] = collisions.at(index);
            const auto bodyA = BodyA.lock();
            const auto bodyB = BodyB.lock();

            if (!bodyA || !bodyB)
                continue;

            const auto rigidBodyA = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyA);
            const auto rigidBodyB = std::dynamic_pointer_cast<Guch2D::RigidBody>(bodyB);

            if (!rigidBodyA || !rigidBodyB)
                continue;

            const auto dynamicBodyA = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyA);
            const auto dynamicBodyB = std::dynamic_pointer_cast<Guch2D::DynamicRigidBody>(bodyB);

            const Guch2D::Vect velocityA = dynamicBodyA ? dynamicBodyA->GetVelocity()
                                                        : Guch2D::Vect {0.0F, 0.0F};
            const Guch2D::Vect velocityB = dynamicBodyB ? dynamicBodyB->GetVelocity()
                                                        : Guch2D::Vect {0.0F, 0.0F};

            const auto deltaVelocity = velocityB - velocityA;

            const auto velocityAlongNormal = Guch2D::VectDot(deltaVelocity, Points.Normal);
            auto& contact = GetSolverContact(Points);

            const float invMassA = GetInverseMass(rigidBodyA);
            const float invMassB = GetInverseMass(rigidBodyB);

            const float invMassSum = invMassA + invMassB;

            if (invMassSum == 0.0F)
                continue;

            // Collision normal is oriented from body B to body A. With rv = (vB - vA),
            // positive value means bodies are approaching along the normal.
            const float restitutionVelocityBias = index < restitutionVelocityBiases.size()
                                                    ? restitutionVelocityBiases.at(index)
                                                    : 0.0F;
            const float normalVelocityError = velocityAlongNormal + restitutionVelocityBias;

            if (normalVelocityError <= 0.0F && contact.AccumulatedNormalImpulse == 0.0F)
                continue;

            const float deltaNormalImpulse = normalVelocityError / invMassSum;
            const float oldNormalImpulse = contact.AccumulatedNormalImpulse;

            contact.AccumulatedNormalImpulse = std::max(oldNormalImpulse + deltaNormalImpulse,
                                                        0.0F);

            const float actualNormalImpulse = contact.AccumulatedNormalImpulse - oldNormalImpulse;

            ApplyImpulse(Points,
                         dynamicBodyA,
                         dynamicBodyB,
                         invMassA,
                         invMassB,
                         actualNormalImpulse,
                         0.0F);

            CalculateTangentialImpulse(Points,
                                       rigidBodyA,
                                       rigidBodyB,
                                       dynamicBodyA,
                                       dynamicBodyB,
                                       deltaVelocity,
                                       invMassA,
                                       invMassB);
        }
    }
}   // namespace

namespace Guch2D
{
    void VelocitySolver::Solve(const std::vector<Collision>& collisions)
    {
        auto currentCollisions = collisions;
        const auto restitutionVelocityBiases = CalculateRestitutionVelocityBiases(
            currentCollisions);
        PrepareWarmStarting(currentCollisions, _previousCollisions);

        constexpr uint8_t totalIterations = 10;
        for (uint8_t iteration = 0; iteration < totalIterations; ++iteration)
        {
            SingleVelocityIteration(currentCollisions, restitutionVelocityBiases);
        }

        RemoveRestitutionFromCachedImpulses(currentCollisions, restitutionVelocityBiases);
        _previousCollisions = std::move(currentCollisions);
    }
}   // namespace Guch2D
