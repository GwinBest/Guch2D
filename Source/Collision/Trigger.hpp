#pragma once

#include <Collision/CollisionBody.hpp>

namespace Guch2D
{
    class Trigger final : public CollisionBody
    {
    public:
        Trigger() noexcept = default;
        Trigger(const Trigger&) = default;
        Trigger(Trigger&&) = default;
        Trigger& operator=(const Trigger&) = default;
        Trigger& operator=(Trigger&&) = default;
        ~Trigger() override = default;
    };
}   // namespace Guch2D
