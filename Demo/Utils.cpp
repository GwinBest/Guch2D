#include "Utils.hpp"

sf::Vector2f ToVector2f(const sf::Vector2i& value)
{
    return {static_cast<float>(value.x), static_cast<float>(value.y)};
}

Guch2D::Vect ScreenToWorld(const sf::Vector2i& screen, const float pixelsPerMeter)
{
    return {static_cast<float>(screen.x) / pixelsPerMeter,
            static_cast<float>(screen.y) / pixelsPerMeter};
}

sf::Vector2f WorldToScreen(const Guch2D::Vect& world, const float pixelsPerMeter)
{
    return {world.x * pixelsPerMeter, world.y * pixelsPerMeter};
}
