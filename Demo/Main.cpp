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
#include "Font.hpp"
#include "Math/Vector.hpp"
#include "Utils.hpp"

namespace
{
    constexpr float PixelsPerMeter = 100.0f;
    constexpr float BallRadiusPixels = 10.0f;
    constexpr float BallMass = 10.0f;
    constexpr float LaunchStrength = 6.0f;
    constexpr float HudScale = 2.0f;
    constexpr float HudPadding = 6.0f;

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
    explicit DynamicsDemo(const float pixelsPerMeter)
        : _pixelsPerMeter(pixelsPerMeter)
    {}

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
    explicit CircleColliderDemo(const float pixelsPerMeter)
        : _pixelsPerMeter(pixelsPerMeter)
    {}

    [[nodiscard]] const char* Name() const noexcept override { return "CIRCLE COLLIDERS"; }

    void Reset(sf::RenderWindow& window) override
    {
        _circles.clear();
        _draggingProbe = false;
        _spawnRadius = 0.25f;
        _probeRadius = 0.35f;
        _world = Guch2D::CollisionWorld();

        const auto size = window.getSize();
        const Guch2D::Vect center = {static_cast<float>(size.x) / _pixelsPerMeter * 0.5f,
                                     static_cast<float>(size.y) / _pixelsPerMeter * 0.5f};
        _probe = CreateCircle(center, _probeRadius);
        _world.AddObject(_probe.body);
    }

    void HandleEvent(const sf::Event& event, sf::RenderWindow& window) override
    {
        if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->code == sf::Keyboard::Key::C)
            {
                _circles.clear();
                _world = Guch2D::CollisionWorld();
                _world.AddObject(_probe.body);
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

                auto collider = std::make_shared<Guch2D::CircleCollider>();
                collider->SetRadius(_spawnRadius);
                auto body = std::make_shared<Guch2D::CollisionBody>(worldPos, collider);
                auto circlePtr = std::make_shared<CircleBody>(
                    CircleBody {body, _spawnRadius, false});

                circlePtr->body->BindOnBeginOverlap([circlePtr](const Guch2D::Collision& callback) {
                    circlePtr->isOverlappingWithProbe = true;
                });
                circlePtr->body->BindOnEndOverlap([circlePtr](const Guch2D::Collision& callback) {
                    circlePtr->isOverlappingWithProbe = false;
                });

                _circles.push_back(circlePtr);
                _world.AddObject(circlePtr->body);
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

        _world.Step();
    }

    void Render(sf::RenderWindow& window) override
    {
        bool anyOverlap = false;

        for (const auto& circlePtr : _circles)
        {
            const bool overlap = circlePtr->isOverlappingWithProbe;
            anyOverlap = anyOverlap || overlap;

            const sf::Color fill = overlap ? sf::Color(220, 80, 80, 140)
                                           : sf::Color(80, 140, 220, 120);
            DrawCircle(window, *circlePtr, fill, sf::Color(255, 255, 255, 80), 1.0f);
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
        bool isOverlappingWithProbe = false;
    };

    using CircleBodyPtr = std::shared_ptr<CircleBody>;

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
        std::size_t count = 0;
        for (const auto& circlePtr : _circles)
        {
            if (circlePtr->isOverlappingWithProbe)
            {
                ++count;
            }
        }
        return count;
    }

    float _pixelsPerMeter = PixelsPerMeter;
    std::vector<CircleBodyPtr> _circles;
    CircleBody _probe;
    Guch2D::CollisionWorld _world;
    bool _draggingProbe = false;
    float _spawnRadius = 0.25f;
    float _probeRadius = 0.35f;
};

class DemoManager
{
public:
    void Add(std::unique_ptr<DemoBase> demo) { _demos.push_back(std::move(demo)); }

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
    lines.push_back("DEMO " + std::to_string(demoIndex + 1) + "/" + std::to_string(demoCount) + ": "
                    + demo.Name());
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
    const sf::Vector2f origin {HudPadding, HudPadding};

    sf::RectangleShape background;
    background.setPosition(origin);
    background.setSize({maxWidth + HudPadding * 2.0f, blockHeight + HudPadding * 2.0f});
    background.setFillColor(sf::Color(0, 0, 0, 180));
    window.draw(background);

    sf::Vector2f cursor {origin.x + HudPadding, origin.y + HudPadding};
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
