#pragma once

#include <cmath>

#include "Dynamics/RigidBody.hpp"
#include "Math/Vector.hpp"

namespace Guch2D
{
    class DynamicRigidBody final : public RigidBody
    {
    public:
        DynamicRigidBody() noexcept = default;

        explicit DynamicRigidBody(const Vect& position) noexcept
            : RigidBody(position)
        {}

        DynamicRigidBody(const Vect& position, const float mass) noexcept
            : RigidBody(position, mass)
        {}

        DynamicRigidBody(const DynamicRigidBody&) = default;
        DynamicRigidBody(DynamicRigidBody&&) = default;
        DynamicRigidBody& operator=(const DynamicRigidBody&) = default;
        DynamicRigidBody& operator=(DynamicRigidBody&&) = default;
        ~DynamicRigidBody() override = default;

        [[nodiscard]] const Vect& GetForce() const noexcept { return _force; }

        void SetForce(const Vect& force) noexcept
        {
            if (!IsFinite(force))
            {
                _force = {0.0F, 0.0F};
                return;
            }

            SetAwake(true);

            _force = force;
        }

        void AddForce(const Vect& force) noexcept
        {
            if (!IsFinite(force))
                return;

            SetAwake(true);

            _force += force;
        }

        void ResetForce() noexcept { _force = Vect(0.0F, 0.0F); }

        [[nodiscard]] const Vect& GetAcceleration() const noexcept { return _acceleration; }

        void SetAcceleration(const Vect& acceleration) noexcept
        {
            if (!IsFinite(acceleration))
            {
                _acceleration = {0.0F, 0.0F};
                return;
            }

            _acceleration = acceleration;
        }

        [[nodiscard]] const Vect& GetVelocity() const noexcept { return _velocity; }

        void SetVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
            {
                _velocity = {0.0F, 0.0F};
                return;
            }

            SetAwake(true);

            _velocity = velocity;
        }

        void AddVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
                return;

            SetAwake(true);

            _velocity += velocity;
        }

        [[nodiscard]] const Vect& GetAngularAcceleration() const noexcept
        {
            return _angularAcceleration;
        }

        void SetAngularAcceleration(const Vect& acceleration) noexcept
        {
            if (!IsFinite(acceleration))
            {
                _angularAcceleration = {0.0F, 0.0F};
                return;
            }

            _angularAcceleration = acceleration;
        }

        [[nodiscard]] const Vect& GetAngularVelocity() const noexcept
        {
            return _angularVelocity;
        }

        void SetAngularVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
            {
                _angularVelocity = {0.0F, 0.0F};
                return;
            }

            SetAwake(true);

            _angularVelocity = velocity;
        }

        void AddAngularVelocity(const Vect& velocity) noexcept
        {
            if (!IsFinite(velocity))
                return;

            SetAwake(true);

            _angularVelocity += velocity;
        }

        [[nodiscard]] const Vect& GetGravityScale() const noexcept { return _gravityScale; }

        void SetGravityScale(const Vect& scale) noexcept
        {
            if (!IsFinite(scale))
            {
                _gravityScale = DefaultGravityScale;
                return;
            }

            _gravityScale = scale;
        }

        [[nodiscard]] const Vect& GetLinearDamping() const noexcept { return _linearDamping; }

        void SetLinearDamping(const Vect& damping) noexcept
        {
            if (!IsFinite(damping))
            {
                _linearDamping = DefaultLinearDamping;
                return;
            }

            _linearDamping = damping;
        }

        [[nodiscard]] const Vect& GetAngularDamping() const noexcept { return _angularDamping; }

        void SetAngularDamping(const Vect& damping) noexcept
        {
            if (!IsFinite(damping))
            {
                _angularDamping = DefaultAngularDamping;
                return;
            }

            _angularDamping = damping;
        }

        [[nodiscard]] bool GetSimulatePhysics() const noexcept { return _simulatePhysics; }

        void SetSimulatePhysics(const bool simulatePhysics) noexcept
        {
            _simulatePhysics = simulatePhysics;
        }

        [[nodiscard]] bool IsAwake() const noexcept { return _isAwake; }

        void SetAwake(const bool awake) noexcept
        {
            if (awake)
            {
                if (!_isAwake)
                {
                    _sleepTime = 0.0F;
                }
                _isAwake = true;
                return;
            }

            _isAwake = false;
            _sleepTime = 0.0F;
            _force = {0.0F, 0.0F};
            _acceleration = {0.0F, 0.0F};
            _velocity = {0.0F, 0.0F};
            _angularAcceleration = {0.0F, 0.0F};
            _angularVelocity = {0.0F, 0.0F};
        }

        [[nodiscard]] float GetSleepTime() const noexcept { return _sleepTime; }

        void ResetSleepTime() noexcept { _sleepTime = 0.0F; }

        void AddSleepTime(const float sleepTime) noexcept
        {
            if (sleepTime <= 0.0F || !std::isfinite(sleepTime))
                return;

            _sleepTime += sleepTime;
        }

    public:
        static constexpr Vect DefaultGravityScale = {1.0F, 1.0F};
        static constexpr Vect DefaultLinearDamping = {0.0F, 0.1F};
        static constexpr Vect DefaultAngularDamping = {0.1F, 0.1F};

    private:
        // Force vector, in Newtons (N)
        Vect _force = {0.0F, 0.0F};

        // Acceleration vector, in meters per second squared (m/s²)
        Vect _acceleration = {0.0F, 0.0F};

        // Velocity vector, in meters per second (m/s)
        Vect _velocity = {0.0F, 0.0F};

        // Angular velocity vector, in radian per second (rad/s)
        Vect _angularVelocity = {0.0F, 0.0F};

        // Angular acceleration vector, in radians per second squared (rad/s²)
        Vect _angularAcceleration = {0.0F, 0.0F};

        // Default gravity scale is 1.0F for both x and y axes
        Vect _gravityScale = DefaultGravityScale;

        // Linear damping vector, in meters per second squared (m/s²)
        // This is used to simulate air resistance
        // Default values are 0.0F for x and 0.1F for y
        Vect _linearDamping = DefaultLinearDamping;

        // Angular damping vector, in radians per second squared (rad/s²)
        // This is used to simulate air resistance
        // Default values are 0.1F for x and 0.1F for y
        Vect _angularDamping = DefaultAngularDamping;

        bool _simulatePhysics = true;

        bool _isAwake = true;

        float _sleepTime = 0.0F;
    };
}   // namespace Guch2D
