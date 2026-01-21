#include <SFML/Graphics.hpp>

#include <array>
#include <cctype>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "Collision/CircleCollider.hpp"
#include "Collision/CollisionBody.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/DynamicWorld.hpp"
#include "Math/Vector.hpp"

namespace
{
    constexpr float PixelsPerMeter = 100.0f;
    constexpr float BallRadiusPixels = 10.0f;
    constexpr float BallMass = 10.0f;
    constexpr float LaunchStrength = 6.0f;
    constexpr float HudScale = 2.0f;
    constexpr float HudPadding = 6.0f;

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

    bool CirclesOverlap(const Guch2D::Vect& a,
                        const float aRadius,
                        const Guch2D::Vect& b,
                        const float bRadius)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float radius = aRadius + bRadius;
        return (dx * dx + dy * dy) <= (radius * radius);
    }

    // Tiny 5x7 ASCII font for the HUD without external font files.
    class PixelFont
    {
    public:
        float MeasureTextWidth(const std::string_view text, const float scale) const
        {
            if (text.empty())
            {
                return 0.0f;
            }

            return (static_cast<float>(text.size()) * (CharWidth + CharSpacing) - CharSpacing)
                   * scale;
        }

        float LineHeight(const float scale) const
        {
            return (CharHeight + LineSpacing) * scale;
        }

        void DrawText(sf::RenderTarget& target,
                      const std::string_view text,
                      const sf::Vector2f position,
                      const float scale,
                      const sf::Color color) const
        {
            sf::RectangleShape pixel({scale, scale});
            pixel.setFillColor(color);

            float x = position.x;
            for (const char c : text)
            {
                const auto& glyph = GetGlyph(c);
                for (int row = 0; row < CharHeight; ++row)
                {
                    for (int col = 0; col < CharWidth; ++col)
                    {
                        const std::uint8_t mask = static_cast<std::uint8_t>(
                            1u << (CharWidth - 1 - col));
                        if ((glyph[row] & mask) == 0u)
                        {
                            continue;
                        }

                        pixel.setPosition(
                            {x + static_cast<float>(col) * scale,
                             position.y + static_cast<float>(row) * scale});
                        target.draw(pixel);
                    }
                }

                x += static_cast<float>(CharWidth + CharSpacing) * scale;
            }
        }

    private:
        static constexpr int CharWidth = 5;
        static constexpr int CharHeight = 7;
        static constexpr int CharSpacing = 1;
        static constexpr int LineSpacing = 2;

        using Glyph = std::array<std::uint8_t, CharHeight>;

        static const Glyph& GetGlyph(char c)
        {
            static const Glyph Empty = {0, 0, 0, 0, 0, 0, 0};

            const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            switch (upper)
            {
            case 'A':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001,
                                            0b10001};
                return glyph;
            }
            case 'B':
            {
                static const Glyph glyph = {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001,
                                            0b11110};
                return glyph;
            }
            case 'C':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001,
                                            0b01110};
                return glyph;
            }
            case 'D':
            {
                static const Glyph glyph = {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001,
                                            0b11110};
                return glyph;
            }
            case 'E':
            {
                static const Glyph glyph = {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000,
                                            0b11111};
                return glyph;
            }
            case 'F':
            {
                static const Glyph glyph = {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000,
                                            0b10000};
                return glyph;
            }
            case 'G':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10000, 0b10000, 0b10011, 0b10001,
                                            0b01110};
                return glyph;
            }
            case 'H':
            {
                static const Glyph glyph = {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001,
                                            0b10001};
                return glyph;
            }
            case 'I':
            {
                static const Glyph glyph = {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
                                            0b11111};
                return glyph;
            }
            case 'J':
            {
                static const Glyph glyph = {0b11111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010,
                                            0b01100};
                return glyph;
            }
            case 'K':
            {
                static const Glyph glyph = {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010,
                                            0b10001};
                return glyph;
            }
            case 'L':
            {
                static const Glyph glyph = {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000,
                                            0b11111};
                return glyph;
            }
            case 'M':
            {
                static const Glyph glyph = {0b10001, 0b11011, 0b10101, 0b10001, 0b10001, 0b10001,
                                            0b10001};
                return glyph;
            }
            case 'N':
            {
                static const Glyph glyph = {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001,
                                            0b10001};
                return glyph;
            }
            case 'O':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001,
                                            0b01110};
                return glyph;
            }
            case 'P':
            {
                static const Glyph glyph = {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000,
                                            0b10000};
                return glyph;
            }
            case 'Q':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010,
                                            0b01101};
                return glyph;
            }
            case 'R':
            {
                static const Glyph glyph = {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010,
                                            0b10001};
                return glyph;
            }
            case 'S':
            {
                static const Glyph glyph = {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001,
                                            0b11110};
                return glyph;
            }
            case 'T':
            {
                static const Glyph glyph = {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
                                            0b00100};
                return glyph;
            }
            case 'U':
            {
                static const Glyph glyph = {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001,
                                            0b01110};
                return glyph;
            }
            case 'V':
            {
                static const Glyph glyph = {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010,
                                            0b00100};
                return glyph;
            }
            case 'W':
            {
                static const Glyph glyph = {0b10001, 0b10001, 0b10001, 0b10001, 0b10101, 0b11011,
                                            0b10001};
                return glyph;
            }
            case 'X':
            {
                static const Glyph glyph = {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001,
                                            0b10001};
                return glyph;
            }
            case 'Y':
            {
                static const Glyph glyph = {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100,
                                            0b00100};
                return glyph;
            }
            case 'Z':
            {
                static const Glyph glyph = {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000,
                                            0b11111};
                return glyph;
            }
            case '0':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001,
                                            0b01110};
                return glyph;
            }
            case '1':
            {
                static const Glyph glyph = {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100,
                                            0b01110};
                return glyph;
            }
            case '2':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000,
                                            0b11111};
                return glyph;
            }
            case '3':
            {
                static const Glyph glyph = {0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001,
                                            0b11110};
                return glyph;
            }
            case '4':
            {
                static const Glyph glyph = {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010,
                                            0b00010};
                return glyph;
            }
            case '5':
            {
                static const Glyph glyph = {0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b00001,
                                            0b11110};
                return glyph;
            }
            case '6':
            {
                static const Glyph glyph = {0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001,
                                            0b01110};
                return glyph;
            }
            case '7':
            {
                static const Glyph glyph = {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000,
                                            0b01000};
                return glyph;
            }
            case '8':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001,
                                            0b01110};
                return glyph;
            }
            case '9':
            {
                static const Glyph glyph = {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010,
                                            0b11100};
                return glyph;
            }
            case ':':
            {
                static const Glyph glyph = {0b00000, 0b00100, 0b00100, 0b00000, 0b00100, 0b00100,
                                            0b00000};
                return glyph;
            }
            case '-':
            {
                static const Glyph glyph = {0b00000, 0b00000, 0b00000, 0b01110, 0b00000, 0b00000,
                                            0b00000};
                return glyph;
            }
            case '.':
            {
                static const Glyph glyph = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00100,
                                            0b00100};
                return glyph;
            }
            case '/':
            {
                static const Glyph glyph = {0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b00000,
                                            0b00000};
                return glyph;
            }
            case ' ':
                return Empty;
            default:
                return Empty;
            }
        }
    };

    std::optional<std::size_t> DemoIndexFromKey(const sf::Keyboard::Key key)
    {
        switch (key)
        {
        case sf::Keyboard::Key::Num1:
        case sf::Keyboard::Key::Numpad1:
            return 0;
        case sf::Keyboard::Key::Num2:
        case sf::Keyboard::Key::Numpad2:
            return 1;
        case sf::Keyboard::Key::Num3:
        case sf::Keyboard::Key::Numpad3:
            return 2;
        case sf::Keyboard::Key::Num4:
        case sf::Keyboard::Key::Numpad4:
            return 3;
        case sf::Keyboard::Key::Num5:
        case sf::Keyboard::Key::Numpad5:
            return 4;
        case sf::Keyboard::Key::Num6:
        case sf::Keyboard::Key::Numpad6:
            return 5;
        case sf::Keyboard::Key::Num7:
        case sf::Keyboard::Key::Numpad7:
            return 6;
        case sf::Keyboard::Key::Num8:
        case sf::Keyboard::Key::Numpad8:
            return 7;
        case sf::Keyboard::Key::Num9:
        case sf::Keyboard::Key::Numpad9:
            return 8;
        default:
            return std::nullopt;
        }
    }
}   // namespace

class DemoBase
{
public:
    virtual ~DemoBase() = default;

    [[nodiscard]] virtual const char* Name() const noexcept = 0;

    virtual void Reset(sf::RenderWindow& window) = 0;
    virtual void HandleEvent(const sf::Event& event, sf::RenderWindow& window) = 0;
    virtual void Update(sf::RenderWindow& window, float dt) = 0;
    virtual void Render(sf::RenderWindow& window) = 0;
    virtual void BuildOverlay(std::vector<std::string>& lines) const = 0;
};

class DynamicsDemo final : public DemoBase
{
public:
    explicit DynamicsDemo(const float pixelsPerMeter) : _pixelsPerMeter(pixelsPerMeter)
    {
    }

    [[nodiscard]] const char* Name() const noexcept override { return "DYNAMICS"; }

    void Reset(sf::RenderWindow& window) override
    {
        (void)window;
        _world = Guch2D::DynamicWorld();
        _bodies.clear();
        _dragging = false;
        _accumulator = 0.0f;
    }

    void HandleEvent(const sf::Event& event, sf::RenderWindow& window) override
    {
        if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->code == sf::Keyboard::Key::C)
            {
                ClearBodies();
            }
            else if (keyPressed->code == sf::Keyboard::Key::R)
            {
                Reset(window);
            }
        }

        if (const auto* mousePressed = event.getIf<sf::Event::MouseButtonPressed>())
        {
            if (mousePressed->button == sf::Mouse::Button::Left)
            {
                _dragging = true;
                _dragStartScreen = ToVector2f(mousePressed->position);
                _dragCurrentScreen = _dragStartScreen;
                _dragStartWorld = ScreenToWorld(mousePressed->position, _pixelsPerMeter);
            }
        }

        if (const auto* mouseReleased = event.getIf<sf::Event::MouseButtonReleased>())
        {
            if (mouseReleased->button == sf::Mouse::Button::Left && _dragging)
            {
                _dragging = false;
                SpawnBody(mouseReleased->position);
            }
        }
    }

    void Update(sf::RenderWindow& window, const float dt) override
    {
        if (_dragging)
        {
            _dragCurrentScreen = ToVector2f(sf::Mouse::getPosition(window));
        }

        _accumulator += dt;
        const float step = _world.GetTimeStep();
        while (_accumulator >= step)
        {
            _world.Step();
            _accumulator -= step;
        }
    }

    void Render(sf::RenderWindow& window) override
    {
        sf::CircleShape shape(BallRadiusPixels);
        shape.setOrigin({BallRadiusPixels, BallRadiusPixels});
        shape.setFillColor(sf::Color(60, 220, 100));

        for (const auto& body : _bodies)
        {
            const auto position = WorldToScreen(body->GetPosition(), _pixelsPerMeter);
            shape.setPosition(position);
            window.draw(shape);
        }

        if (_dragging)
        {
            sf::VertexArray line(sf::PrimitiveType::Lines, 2);
            line[0] = sf::Vertex(_dragStartScreen, sf::Color(255, 216, 0));
            line[1] = sf::Vertex(_dragCurrentScreen, sf::Color(255, 216, 0));
            window.draw(line);
        }
    }

    void BuildOverlay(std::vector<std::string>& lines) const override
    {
        lines.push_back("LEFT MOUSE DRAG: SPAWN BALL");
        lines.push_back("C: CLEAR BALLS");
        lines.push_back("R: RESET DEMO");
        lines.push_back("BALLS: " + std::to_string(_bodies.size()));
    }

private:
    void SpawnBody(const sf::Vector2i& releasePosition)
    {
        const auto releaseWorld = ScreenToWorld(releasePosition, _pixelsPerMeter);
        const auto launchVector = _dragStartWorld - releaseWorld;

        auto body = std::make_shared<Guch2D::DynamicRigidBody>(_dragStartWorld, BallMass);
        body->SetVelocity(launchVector * LaunchStrength);

        _world.AddObject(body);
        _bodies.push_back(body);
    }

    void ClearBodies()
    {
        _world = Guch2D::DynamicWorld();
        _bodies.clear();
        _accumulator = 0.0f;
    }

    float _pixelsPerMeter = PixelsPerMeter;
    Guch2D::DynamicWorld _world;
    std::vector<std::shared_ptr<Guch2D::DynamicRigidBody>> _bodies;
    bool _dragging = false;
    Guch2D::Vect _dragStartWorld = {0.0f, 0.0f};
    sf::Vector2f _dragStartScreen = {0.0f, 0.0f};
    sf::Vector2f _dragCurrentScreen = {0.0f, 0.0f};
    float _accumulator = 0.0f;
};

class CircleColliderDemo final : public DemoBase
{
public:
    explicit CircleColliderDemo(const float pixelsPerMeter) : _pixelsPerMeter(pixelsPerMeter)
    {
    }

    [[nodiscard]] const char* Name() const noexcept override { return "CIRCLE COLLIDERS"; }

    void Reset(sf::RenderWindow& window) override
    {
        _circles.clear();
        _draggingProbe = false;
        _spawnRadius = 0.25f;
        _probeRadius = 0.35f;

        const auto size = window.getSize();
        const Guch2D::Vect center = {
            static_cast<float>(size.x) / _pixelsPerMeter * 0.5f,
            static_cast<float>(size.y) / _pixelsPerMeter * 0.5f};
        _probe = CreateCircle(center, _probeRadius);
    }

    void HandleEvent(const sf::Event& event, sf::RenderWindow& window) override
    {
        if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->code == sf::Keyboard::Key::C)
            {
                _circles.clear();
            }
            else if (keyPressed->code == sf::Keyboard::Key::R)
            {
                Reset(window);
            }
        }

        if (const auto* mousePressed = event.getIf<sf::Event::MouseButtonPressed>())
        {
            if (mousePressed->button == sf::Mouse::Button::Left)
            {
                const auto worldPos = ScreenToWorld(mousePressed->position, _pixelsPerMeter);
                _circles.push_back(CreateCircle(worldPos, _spawnRadius));
            }
            else if (mousePressed->button == sf::Mouse::Button::Right)
            {
                _draggingProbe = true;
                _probe.body->SetPosition(ScreenToWorld(mousePressed->position, _pixelsPerMeter));
            }
        }

        if (const auto* mouseReleased = event.getIf<sf::Event::MouseButtonReleased>())
        {
            if (mouseReleased->button == sf::Mouse::Button::Right)
            {
                _draggingProbe = false;
            }
        }
    }

    void Update(sf::RenderWindow& window, float dt) override
    {
        (void)dt;
        if (_draggingProbe)
        {
            _probe.body->SetPosition(
                ScreenToWorld(sf::Mouse::getPosition(window), _pixelsPerMeter));
        }
    }

    void Render(sf::RenderWindow& window) override
    {
        const auto probeCenter = _probe.body->GetColliderCenterWorld();
        bool anyOverlap = false;

        for (const auto& circle : _circles)
        {
            const auto circleCenter = circle.body->GetColliderCenterWorld();
            const bool overlap = CirclesOverlap(probeCenter, _probe.radius, circleCenter, circle.radius);
            anyOverlap = anyOverlap || overlap;

            const sf::Color fill = overlap ? sf::Color(220, 80, 80, 140)
                                           : sf::Color(80, 140, 220, 120);
            DrawCircle(window, circle, fill, sf::Color(255, 255, 255, 80), 1.0f);
        }

        const sf::Color probeColor = anyOverlap ? sf::Color(255, 80, 80, 160)
                                                : sf::Color(80, 240, 120, 160);
        DrawCircle(window, _probe, probeColor, sf::Color(255, 255, 255, 200), 2.0f);
    }

    void BuildOverlay(std::vector<std::string>& lines) const override
    {
        lines.push_back("LEFT MOUSE: ADD CIRCLE");
        lines.push_back("RIGHT MOUSE DRAG: MOVE PROBE");
        lines.push_back("C: CLEAR CIRCLES");
        lines.push_back("R: RESET DEMO");
        lines.push_back("CIRCLES: " + std::to_string(_circles.size()));
        lines.push_back("OVERLAPS: " + std::to_string(CountOverlaps()));
    }

private:
    struct CircleBody
    {
        std::shared_ptr<Guch2D::CollisionBody> body;
        float radius = 0.0f;
    };

    CircleBody CreateCircle(const Guch2D::Vect& position, const float radius)
    {
        auto collider = std::make_shared<Guch2D::CircleCollider>();
        collider->SetRadius(radius);

        auto body = std::make_shared<Guch2D::CollisionBody>(position, collider);
        return {body, radius};
    }

    void DrawCircle(sf::RenderWindow& window,
                    const CircleBody& circle,
                    const sf::Color fill,
                    const sf::Color outline,
                    const float outlineThickness) const
    {
        const float radiusPixels = circle.radius * _pixelsPerMeter;
        sf::CircleShape shape(radiusPixels);
        shape.setOrigin({radiusPixels, radiusPixels});
        shape.setPosition(WorldToScreen(circle.body->GetColliderCenterWorld(), _pixelsPerMeter));
        shape.setFillColor(fill);
        shape.setOutlineColor(outline);
        shape.setOutlineThickness(outlineThickness);
        window.draw(shape);
    }

    std::size_t CountOverlaps() const
    {
        const auto probeCenter = _probe.body->GetColliderCenterWorld();
        std::size_t count = 0;
        for (const auto& circle : _circles)
        {
            if (CirclesOverlap(probeCenter, _probe.radius,
                               circle.body->GetColliderCenterWorld(), circle.radius))
            {
                ++count;
            }
        }
        return count;
    }

    float _pixelsPerMeter = PixelsPerMeter;
    std::vector<CircleBody> _circles;
    CircleBody _probe;
    bool _draggingProbe = false;
    float _spawnRadius = 0.25f;
    float _probeRadius = 0.35f;
};

class DemoManager
{
public:
    void Add(std::unique_ptr<DemoBase> demo)
    {
        _demos.push_back(std::move(demo));
    }

    DemoBase& Active() { return *_demos.at(_activeIndex); }
    [[nodiscard]] const DemoBase& Active() const { return *_demos.at(_activeIndex); }

    [[nodiscard]] std::size_t Count() const { return _demos.size(); }
    [[nodiscard]] std::size_t ActiveIndex() const { return _activeIndex; }

    void ResetActive(sf::RenderWindow& window)
    {
        if (_demos.empty())
        {
            return;
        }

        _demos[_activeIndex]->Reset(window);
    }

    void SetActive(const std::size_t index, sf::RenderWindow& window)
    {
        if (index >= _demos.size() || index == _activeIndex)
        {
            return;
        }

        _activeIndex = index;
        _demos[_activeIndex]->Reset(window);
        UpdateTitle(window);
    }

    void UpdateTitle(sf::RenderWindow& window) const
    {
        if (_demos.empty())
        {
            return;
        }

        std::string title = "Guch2D - Demo ";
        title += std::to_string(_activeIndex + 1);
        title += ": ";
        title += Active().Name();
        window.setTitle(title);
    }

private:
    std::vector<std::unique_ptr<DemoBase>> _demos;
    std::size_t _activeIndex = 0;
};

void RenderOverlay(sf::RenderWindow& window,
                   const PixelFont& font,
                   const DemoBase& demo,
                   const std::size_t demoIndex,
                   const std::size_t demoCount)
{
    std::vector<std::string> lines;
    lines.push_back("DEMO " + std::to_string(demoIndex + 1) + "/" + std::to_string(demoCount) +
                    ": " + demo.Name());
    lines.push_back("1-9: SWITCH DEMO");
    lines.push_back("ESC: QUIT");
    lines.push_back("");
    demo.BuildOverlay(lines);

    float maxWidth = 0.0f;
    for (const auto& line : lines)
    {
        const float width = font.MeasureTextWidth(line, HudScale);
        if (width > maxWidth)
        {
            maxWidth = width;
        }
    }

    const float lineHeight = font.LineHeight(HudScale);
    const float blockHeight = lineHeight * static_cast<float>(lines.size());
    const sf::Vector2f origin{HudPadding, HudPadding};

    sf::RectangleShape background;
    background.setPosition(origin);
    background.setSize({maxWidth + HudPadding * 2.0f, blockHeight + HudPadding * 2.0f});
    background.setFillColor(sf::Color(0, 0, 0, 180));
    window.draw(background);

    sf::Vector2f cursor{origin.x + HudPadding, origin.y + HudPadding};
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        if (lines[i].empty())
        {
            cursor.y += lineHeight;
            continue;
        }

        const sf::Color color = (i == 0) ? sf::Color(255, 216, 0) : sf::Color(230, 230, 230);
        font.DrawText(window, lines[i], cursor, HudScale, color);
        cursor.y += lineHeight;
    }
}

int main()
{
    sf::RenderWindow window(sf::VideoMode({1280, 720}), "Guch2D");
    window.setFramerateLimit(144);

    DemoManager demos;
    demos.Add(std::make_unique<DynamicsDemo>(PixelsPerMeter));
    demos.Add(std::make_unique<CircleColliderDemo>(PixelsPerMeter));
    demos.ResetActive(window);
    demos.UpdateTitle(window);

    PixelFont font;
    sf::Clock clock;

    while (window.isOpen())
    {
        const float dt = clock.restart().asSeconds();

        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
                continue;
            }

            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
            {
                if (keyPressed->code == sf::Keyboard::Key::Escape)
                {
                    window.close();
                    continue;
                }

                if (const auto demoIndex = DemoIndexFromKey(keyPressed->code))
                {
                    demos.SetActive(*demoIndex, window);
                    continue;
                }
            }

            demos.Active().HandleEvent(*event, window);
        }

        demos.Active().Update(window, dt);

        window.clear(sf::Color::Black);
        demos.Active().Render(window);
        RenderOverlay(window, font, demos.Active(), demos.ActiveIndex(), demos.Count());
        window.display();
    }

    return 0;
}
