#pragma once

#include "Math/Vector.hpp"
#include "SFML/System/Vector2.hpp"

sf::Vector2f ToVector2f(const sf::Vector2i& value);

Guch2D::Vect ScreenToWorld(const sf::Vector2i& screen, const float pixelsPerMeter);

sf::Vector2f WorldToScreen(const Guch2D::Vect& world, const float pixelsPerMeter);
