#pragma once

#include <string_view>

#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/RectangleShape.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/System/Vector2.hpp"
#include "SFML/Window/Keyboard.hpp"

// Tiny 5x7 ASCII font for the HUD without external font files.
class PixelFont
{
public:
    static float MeasureTextWidth(const std::string_view text, const float scale)
    {
        if (text.empty())
        {
            return 0.0f;
        }

        return (static_cast<float>(text.size()) * (_charWidth + _charSpacing) - _charSpacing)
             * scale;
    }

    static float LineHeight(const float scale) { return (_charHeight + _lineSpacing) * scale; }

    static void DrawText(sf::RenderTarget& target,
                         const std::string_view text,
                         const sf::Vector2f position,
                         const float scale,
                         const sf::Color color)
    {
        sf::RectangleShape pixel({scale, scale});
        pixel.setFillColor(color);

        float x = position.x;
        for (const char c : text)
        {
            const auto& glyph = GetGlyph(c);
            for (int row = 0; row < _charHeight; ++row)
            {
                for (int col = 0; col < _charWidth; ++col)
                {
                    const std::uint8_t mask = static_cast<std::uint8_t>(1u
                                                                        << (_charWidth - 1 - col));
                    if ((glyph[static_cast<size_t>(row)] & mask) == 0u)
                    {
                        continue;
                    }

                    pixel.setPosition({x + static_cast<float>(col) * scale,
                                       position.y + static_cast<float>(row) * scale});
                    target.draw(pixel);
                }
            }

            x += static_cast<float>(_charWidth + _charSpacing) * scale;
        }
    }

private:
    static constexpr int _charWidth = 5;
    static constexpr int _charHeight = 7;
    static constexpr int _charSpacing = 1;
    static constexpr int _lineSpacing = 2;

    using Glyph = std::array<std::uint8_t, _charHeight>;

    struct GlyphEntry
    {
        char Key;
        Glyph Data;
    };

    static const Glyph& GetGlyph(const char c)
    {
        static constexpr Glyph Empty = {0, 0, 0, 0, 0, 0, 0};
        static constexpr std::array<GlyphEntry, 40> Glyphs = {
            {
             {'A', {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
             {'B', {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110}},
             {'C', {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110}},
             {'D', {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110}},
             {'E', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}},
             {'F', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000}},
             {'G', {0b01110, 0b10001, 0b10000, 0b10000, 0b10011, 0b10001, 0b01110}},
             {'H', {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
             {'I', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111}},
             {'J', {0b11111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100}},
             {'K', {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}},
             {'L', {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}},
             {'M', {0b10001, 0b11011, 0b10101, 0b10001, 0b10001, 0b10001, 0b10001}},
             {'N', {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001}},
             {'O', {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
             {'P', {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}},
             {'Q', {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101}},
             {'R', {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}},
             {'S', {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}},
             {'T', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
             {'U', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
             {'V', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100}},
             {'W', {0b10001, 0b10001, 0b10001, 0b10001, 0b10101, 0b11011, 0b10001}},
             {'X', {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001}},
             {'Y', {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100}},
             {'Z', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111}},
             {'0', {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110}},
             {'1', {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
             {'2', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}},
             {'3', {0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110}},
             {'4', {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}},
             {'5', {0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b00001, 0b11110}},
             {'6', {0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}},
             {'7', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}},
             {'8', {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}},
             {'9', {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b11100}},
             {':', {0b00000, 0b00100, 0b00100, 0b00000, 0b00100, 0b00100, 0b00000}},
             {'-', {0b00000, 0b00000, 0b00000, 0b01110, 0b00000, 0b00000, 0b00000}},
             {'.', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00100, 0b00100}},
             {'/', {0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b00000, 0b00000}},
             }
        };

        const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        for (const auto& [Key, Data] : Glyphs)
        {
            if (Key == upper)
            {
                return Data;
            }
        }

        return Empty;
    }
};

std::optional<std::size_t> DemoIndexFromKey(const sf::Keyboard::Key key)
{
    static constexpr std::array<std::array<sf::Keyboard::Key, 2>, 9> keyMap = {
        {
         {sf::Keyboard::Key::Num1, sf::Keyboard::Key::Numpad1},
         {sf::Keyboard::Key::Num2, sf::Keyboard::Key::Numpad2},
         {sf::Keyboard::Key::Num3, sf::Keyboard::Key::Numpad3},
         {sf::Keyboard::Key::Num4, sf::Keyboard::Key::Numpad4},
         {sf::Keyboard::Key::Num5, sf::Keyboard::Key::Numpad5},
         {sf::Keyboard::Key::Num6, sf::Keyboard::Key::Numpad6},
         {sf::Keyboard::Key::Num7, sf::Keyboard::Key::Numpad7},
         {sf::Keyboard::Key::Num8, sf::Keyboard::Key::Numpad8},
         {sf::Keyboard::Key::Num9, sf::Keyboard::Key::Numpad9},
         }
    };

    for (std::size_t i = 0; i < keyMap.size(); ++i)
    {
        if (key == keyMap[i][0] || key == keyMap[i][1])
        {
            return i;
        }
    }

    return std::nullopt;
}
