#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_set>
#include <string>
#include <string_view>
#include <vector>

#include "Collision/AABBCollider.hpp"
#include "Collision/CircleCollider.hpp"
#include "Collision/CollisionBody.hpp"
#include "Collision/CollisionWorld.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/DynamicWorld.hpp"
#include "Dynamics/KinematicBody.hpp"
#include "Dynamics/StaticRigidBody.hpp"
#include "Font.hpp"
#include "Math/Vector.hpp"
#include "Solver/VelocitySolver.hpp"
#include "Utils.hpp"

namespace
{
    constexpr float PixelsPerMeter = 100.0f;
    constexpr float BallRadiusPixels = 10.0f;
    constexpr float BallMass = 10.0f;
    constexpr float LaunchStrength = 6.0f;
    constexpr float HudScale = 2.0f;
    constexpr float HudPadding = 6.0f;
    constexpr float CollisionNormalLengthMeters = 0.35f;
    constexpr float CollisionNormalHeadLengthMeters = 0.12f;
    constexpr float CollisionNormalHeadWidthMeters = 0.08f;
    constexpr float CollisionPointRadiusPixels = 4.0f;

    class CollisionWorldInspector final : public Guch2D::CollisionWorld
    {
    public:
        [[nodiscard]] static Guch2D::CollisionPoints
            Check(const std::shared_ptr<Guch2D::CollisionBody>& bodyA,
                  const std::shared_ptr<Guch2D::CollisionBody>& bodyB)
        {
            return CheckCollisions(bodyA, bodyB);
        }
    };

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

class BoundedDynamicsDemo final : public DemoBase
{
public:
    explicit BoundedDynamicsDemo(const float pixelsPerMeter)
        : _pixelsPerMeter(pixelsPerMeter)
    {}

    [[nodiscard]] const char* Name() const noexcept override { return "DYNAMICS BOUNDS"; }

    void Reset(sf::RenderWindow& window) override
    {
        _dynamicBodies.clear();
        _staticBodies.clear();
        _dragging = false;
        _accumulator = 0.0f;
        _spawnShape = SpawnShape::Circle;
        _spawnMass = BallMass;
        _spawnBounciness = DefaultSpawnBounciness;
        _spawnFriction = DefaultSpawnFriction;
        _spawnCircleRadius = BallRadiusPixels / _pixelsPerMeter;
        _spawnAABBExtent = {0.12f, 0.12f};

        BuildStaticBounds(window);
        RebuildWorld();
    }

    void HandleEvent(const sf::Event& event, sf::RenderWindow& window) override
    {
        if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>())
        {
            switch (keyPressed->code)
            {
            case sf::Keyboard::Key::C:   ClearDynamicBodies(); break;
            case sf::Keyboard::Key::R:   Reset(window); break;
            case sf::Keyboard::Key::Tab: ToggleSpawnShape(); break;
            case sf::Keyboard::Key::Q:   AdjustSpawnMass(MassStep); break;
            case sf::Keyboard::Key::A:   AdjustSpawnMass(-MassStep); break;
            case sf::Keyboard::Key::Z:   AdjustSpawnBounciness(BouncinessStep); break;
            case sf::Keyboard::Key::X:   AdjustSpawnBounciness(-BouncinessStep); break;
            case sf::Keyboard::Key::F:   AdjustSpawnFriction(FrictionStep); break;
            case sf::Keyboard::Key::G:   AdjustSpawnFriction(-FrictionStep); break;
            case sf::Keyboard::Key::W:   AdjustSpawnPrimarySize(SizeStep); break;
            case sf::Keyboard::Key::S:   AdjustSpawnPrimarySize(-SizeStep); break;
            case sf::Keyboard::Key::E:   AdjustSpawnAABBHeight(SizeStep); break;
            case sf::Keyboard::Key::D:   AdjustSpawnAABBHeight(-SizeStep); break;
            default:                     break;
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
        std::unordered_set<const Guch2D::CollisionBody*> collidingBodies;
        collidingBodies.reserve(_dynamicBodies.size());

        for (size_t i = 0; i < _dynamicBodies.size(); ++i)
        {
            for (size_t j = i + 1; j < _dynamicBodies.size(); ++j)
            {
                const auto points = CollisionWorldInspector::Check(_dynamicBodies[i].body,
                                                                   _dynamicBodies[j].body);
                if (!points.HasCollision)
                    continue;

                collidingBodies.insert(_dynamicBodies[i].body.get());
                collidingBodies.insert(_dynamicBodies[j].body.get());
            }

            for (const auto& staticBody : _staticBodies)
            {
                const auto points =
                    CollisionWorldInspector::Check(_dynamicBodies[i].body, staticBody.body);
                if (!points.HasCollision)
                    continue;

                collidingBodies.insert(_dynamicBodies[i].body.get());
                break;
            }
        }

        for (const auto& staticBody : _staticBodies)
        {
            DrawAABB(window,
                     staticBody.body->GetColliderCenterWorld(),
                     staticBody.extent,
                     sf::Color(110, 110, 110, 220),
                     sf::Color(170, 170, 170, 255),
                     1.0f);
        }

        for (const auto& body : _dynamicBodies)
        {
            const bool isColliding = collidingBodies.contains(body.body.get());
            const bool isSleeping = !body.body->IsAwake();
            const sf::Color fillColor =
                isColliding ? sf::Color(230, 70, 70, 185)
                            : (body.shape == SpawnShape::Circle ? sf::Color(70, 220, 120, 180)
                                                                : sf::Color(80, 170, 255, 180));
            const sf::Color outlineColor = isSleeping ? sf::Color(255, 210, 70, 245)
                                                      : sf::Color(255, 255, 255, 220);

            if (body.shape == SpawnShape::Circle)
            {
                DrawCircle(window,
                           body.body->GetColliderCenterWorld(),
                           body.radius,
                           fillColor,
                           outlineColor,
                           1.0f);
                continue;
            }

            DrawAABB(window,
                     body.body->GetColliderCenterWorld(),
                     body.extent,
                     fillColor,
                     outlineColor,
                     1.0f);
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
        const size_t sleepingBodies =
            static_cast<size_t>(std::count_if(_dynamicBodies.begin(),
                                              _dynamicBodies.end(),
                                              [](const DynamicBodyVisual& dynamicBody) {
                                                  return !dynamicBody.body->IsAwake();
                                              }));

        lines.emplace_back("LEFT MOUSE DRAG: SPAWN BODY");
        lines.emplace_back("TAB: TOGGLE SPAWN TYPE");
        lines.emplace_back("Q/A: MASS UP/DOWN");
        lines.emplace_back("Z/X: BOUNCINESS UP/DOWN");
        lines.emplace_back("F/G: FRICTION UP/DOWN");
        lines.emplace_back("W/S: WIDTH OR RADIUS UP/DOWN");
        lines.emplace_back("E/D: AABB HEIGHT UP/DOWN");
        lines.emplace_back("C: CLEAR DYNAMIC BODIES");
        lines.emplace_back("R: RESET DEMO");
        lines.emplace_back("SPAWN: " + std::string(SpawnShapeName()));
        lines.emplace_back("MASS: " + std::to_string(_spawnMass));
        lines.emplace_back("BOUNCINESS: " + std::to_string(_spawnBounciness));
        lines.emplace_back("FRICTION: " + std::to_string(_spawnFriction));
        lines.emplace_back("BALL RADIUS: " + std::to_string(_spawnCircleRadius));
        lines.emplace_back("AABB X: " + std::to_string(_spawnAABBExtent.x));
        lines.emplace_back("AABB Y: " + std::to_string(_spawnAABBExtent.y));
        lines.emplace_back("DYNAMIC BODIES: " + std::to_string(_dynamicBodies.size()));
        lines.emplace_back("SLEEPING BODIES: " + std::to_string(sleepingBodies));
    }

private:
    enum class SpawnShape : std::uint8_t
    {
        Circle,
        AABB,
    };

    struct DynamicBodyVisual
    {
        std::shared_ptr<Guch2D::DynamicRigidBody> body;
        SpawnShape shape = SpawnShape::Circle;
        float radius = 0.0f;
        Guch2D::Vect extent = {0.0f, 0.0f};
    };

    struct StaticBodyVisual
    {
        std::shared_ptr<Guch2D::StaticRigidBody> body;
        Guch2D::Vect extent = {0.0f, 0.0f};
    };

    [[nodiscard]] StaticBodyVisual CreateStaticAABB(const Guch2D::Vect& position,
                                                    const Guch2D::Vect& extent) const
    {
        auto collider = std::make_shared<Guch2D::AABBCollider>();
        collider->SetExtent(extent);

        auto body = std::make_shared<Guch2D::StaticRigidBody>(position);
        body->SetCollider(collider);
        body->SetBounciness(1.0f);
        body->SetStaticFriction(_spawnFriction);
        body->SetDynamicFriction(_spawnFriction);
        return {body, extent};
    }

    void BuildStaticBounds(const sf::RenderWindow& window)
    {
        const auto size = window.getSize();
        const float worldWidth = static_cast<float>(size.x) / _pixelsPerMeter;
        const float worldHeight = static_cast<float>(size.y) / _pixelsPerMeter;

        constexpr float wallThickness = 0.28f;
        constexpr float floorThickness = 0.28f;

        const float halfWallThickness = wallThickness * 0.5f;
        const float halfFloorThickness = floorThickness * 0.5f;

        _staticBodies.push_back(
            CreateStaticAABB({worldWidth * 0.5f, worldHeight - halfFloorThickness},
                             {worldWidth * 0.5f, halfFloorThickness}));
        _staticBodies.push_back(CreateStaticAABB({worldWidth * 0.5f, halfFloorThickness},
                                                 {worldWidth * 0.5f, halfFloorThickness}));
        _staticBodies.push_back(CreateStaticAABB({halfWallThickness, worldHeight * 0.5f},
                                                 {halfWallThickness, worldHeight * 0.5f}));
        _staticBodies.push_back(
            CreateStaticAABB({worldWidth - halfWallThickness, worldHeight * 0.5f},
                             {halfWallThickness, worldHeight * 0.5f}));
    }

    void RebuildWorld()
    {
        _world = Guch2D::DynamicWorld();
        _world.AddSolver(std::make_shared<Guch2D::PositionSolver>());
        _world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());

        for (const auto& staticBody : _staticBodies)
        {
            _world.AddObject(staticBody.body);
        }

        for (const auto& dynamicBody : _dynamicBodies)
        {
            _world.AddObject(dynamicBody.body);
        }
    }

    void ClearDynamicBodies()
    {
        _dynamicBodies.clear();
        _accumulator = 0.0f;
        RebuildWorld();
    }

    void ToggleSpawnShape()
    {
        _spawnShape = (_spawnShape == SpawnShape::Circle) ? SpawnShape::AABB : SpawnShape::Circle;
    }

    void AdjustSpawnMass(const float delta)
    {
        _spawnMass = std::clamp(_spawnMass + delta, MinSpawnMass, MaxSpawnMass);
    }

    void AdjustSpawnBounciness(const float delta)
    {
        _spawnBounciness = std::clamp(_spawnBounciness + delta,
                                      MinSpawnBounciness,
                                      MaxSpawnBounciness);

        for (const auto& dynamicBody : _dynamicBodies)
        {
            dynamicBody.body->SetBounciness(_spawnBounciness);
        }
    }

    void AdjustSpawnFriction(const float delta)
    {
        _spawnFriction = std::clamp(_spawnFriction + delta, MinSpawnFriction, MaxSpawnFriction);

        for (const auto& dynamicBody : _dynamicBodies)
        {
            dynamicBody.body->SetStaticFriction(_spawnFriction);
            dynamicBody.body->SetDynamicFriction(_spawnFriction);
        }

        for (const auto& staticBody : _staticBodies)
        {
            staticBody.body->SetStaticFriction(_spawnFriction);
            staticBody.body->SetDynamicFriction(_spawnFriction);
        }
    }

    void AdjustSpawnPrimarySize(const float delta)
    {
        if (_spawnShape == SpawnShape::Circle)
        {
            _spawnCircleRadius = std::clamp(_spawnCircleRadius + delta,
                                            MinSpawnRadius,
                                            MaxSpawnRadius);
            return;
        }

        _spawnAABBExtent.x = std::clamp(_spawnAABBExtent.x + delta, MinSpawnExtent, MaxSpawnExtent);
    }

    void AdjustSpawnAABBHeight(const float delta)
    {
        _spawnAABBExtent.y = std::clamp(_spawnAABBExtent.y + delta, MinSpawnExtent, MaxSpawnExtent);
    }

    void SpawnBody(const sf::Vector2i& releasePosition)
    {
        const auto releaseWorld = ScreenToWorld(releasePosition, _pixelsPerMeter);
        const auto launchVector = _dragStartWorld - releaseWorld;
        const auto launchVelocity = launchVector * LaunchStrength;

        if (_spawnShape == SpawnShape::Circle)
        {
            SpawnCircle(_dragStartWorld, launchVelocity);
            return;
        }

        SpawnAABB(_dragStartWorld, launchVelocity);
    }

    void SpawnCircle(const Guch2D::Vect& position, const Guch2D::Vect& launchVelocity)
    {
        auto collider = std::make_shared<Guch2D::CircleCollider>();
        collider->SetRadius(_spawnCircleRadius);

        auto body = std::make_shared<Guch2D::DynamicRigidBody>(position, _spawnMass);
        body->SetCollider(collider);
        body->SetBounciness(_spawnBounciness);
        body->SetStaticFriction(_spawnFriction);
        body->SetDynamicFriction(_spawnFriction);
        body->SetVelocity(launchVelocity);

        _world.AddObject(body);
        _dynamicBodies.push_back({
            body,
            SpawnShape::Circle,
            _spawnCircleRadius,
            {0.0f, 0.0f}
        });
    }

    void SpawnAABB(const Guch2D::Vect& position, const Guch2D::Vect& launchVelocity)
    {
        auto collider = std::make_shared<Guch2D::AABBCollider>();
        collider->SetExtent(_spawnAABBExtent);

        auto body = std::make_shared<Guch2D::DynamicRigidBody>(position, _spawnMass);
        body->SetCollider(collider);
        body->SetBounciness(_spawnBounciness);
        body->SetStaticFriction(_spawnFriction);
        body->SetDynamicFriction(_spawnFriction);
        body->SetVelocity(launchVelocity);

        _world.AddObject(body);
        _dynamicBodies.push_back({body, SpawnShape::AABB, 0.0f, _spawnAABBExtent});
    }

    void DrawCircle(sf::RenderWindow& window,
                    const Guch2D::Vect& center,
                    const float radius,
                    const sf::Color fill,
                    const sf::Color outline,
                    const float outlineThickness) const
    {
        const float radiusPixels = radius * _pixelsPerMeter;
        sf::CircleShape shape(radiusPixels);
        shape.setOrigin({radiusPixels, radiusPixels});
        shape.setPosition(WorldToScreen(center, _pixelsPerMeter));
        shape.setFillColor(fill);
        shape.setOutlineColor(outline);
        shape.setOutlineThickness(outlineThickness);
        window.draw(shape);
    }

    void DrawAABB(sf::RenderWindow& window,
                  const Guch2D::Vect& center,
                  const Guch2D::Vect& extent,
                  const sf::Color fill,
                  const sf::Color outline,
                  const float outlineThickness) const
    {
        const sf::Vector2f halfExtentPixels = {extent.x * _pixelsPerMeter,
                                               extent.y * _pixelsPerMeter};

        sf::RectangleShape shape(halfExtentPixels * 2.0f);
        shape.setOrigin(halfExtentPixels);
        shape.setPosition(WorldToScreen(center, _pixelsPerMeter));
        shape.setFillColor(fill);
        shape.setOutlineColor(outline);
        shape.setOutlineThickness(outlineThickness);
        window.draw(shape);
    }

    [[nodiscard]] const char* SpawnShapeName() const noexcept
    {
        return _spawnShape == SpawnShape::Circle ? "BALL" : "AABB";
    }

    static constexpr float MinSpawnMass = 0.1f;
    static constexpr float MaxSpawnMass = 100.0f;
    static constexpr float MassStep = 0.5f;
    static constexpr float MinSpawnBounciness = 0.0f;
    static constexpr float MaxSpawnBounciness = 1.0f;
    static constexpr float DefaultSpawnBounciness = 0.5f;
    static constexpr float BouncinessStep = 0.05f;
    static constexpr float MinSpawnFriction = 0.0f;
    static constexpr float MaxSpawnFriction = 1.0f;
    static constexpr float DefaultSpawnFriction = 1.0f;
    static constexpr float FrictionStep = 0.05f;
    static constexpr float MinSpawnRadius = 0.04f;
    static constexpr float MaxSpawnRadius = 0.8f;
    static constexpr float MinSpawnExtent = 0.04f;
    static constexpr float MaxSpawnExtent = 0.8f;
    static constexpr float SizeStep = 0.02f;

    float _pixelsPerMeter = PixelsPerMeter;
    Guch2D::DynamicWorld _world;
    std::vector<DynamicBodyVisual> _dynamicBodies;
    std::vector<StaticBodyVisual> _staticBodies;
    bool _dragging = false;
    Guch2D::Vect _dragStartWorld = {0.0f, 0.0f};
    sf::Vector2f _dragStartScreen = {0.0f, 0.0f};
    sf::Vector2f _dragCurrentScreen = {0.0f, 0.0f};
    SpawnShape _spawnShape = SpawnShape::Circle;
    float _spawnMass = BallMass;
    float _spawnBounciness = DefaultSpawnBounciness;
    float _spawnFriction = DefaultSpawnFriction;
    float _spawnCircleRadius = BallRadiusPixels / PixelsPerMeter;
    Guch2D::Vect _spawnAABBExtent = {0.12f, 0.12f};
    float _accumulator = 0.0f;
};

class CircleColliderDemo final : public DemoBase
{
public:
    explicit CircleColliderDemo(const float pixelsPerMeter)
        : _pixelsPerMeter(pixelsPerMeter)
    {}

    [[nodiscard]] const char* Name() const noexcept override { return "COLLIDERS"; }

    void Reset(sf::RenderWindow& window) override
    {
        _circles.clear();
        _aabbs.clear();
        _collisionPairs.clear();
        _draggingProbe = false;
        _spawnAABBMode = false;
        _spawnRadius = 0.25f;
        _probeRadius = 0.35f;
        _spawnAABBExtent = {0.3f, 0.2f};
        _world = Guch2D::DynamicWorld();

        const auto size = window.getSize();
        const Guch2D::Vect center = {static_cast<float>(size.x) / _pixelsPerMeter * 0.5f,
                                     static_cast<float>(size.y) / _pixelsPerMeter * 0.5f};
        _probe = CreateCircle(center, _probeRadius);
        BindCollisionCallbacks(_probe.body, _probe.overlapCount);
        ResetWorldWithProbe();
    }

    void HandleEvent(const sf::Event& event, sf::RenderWindow& window) override
    {
        if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->code == sf::Keyboard::Key::C)
            {
                _circles.clear();
                _aabbs.clear();
                _collisionPairs.clear();
                _probe.overlapCount = 0;
                ResetWorldWithProbe();
            }
            else if (keyPressed->code == sf::Keyboard::Key::R)
            {
                Reset(window);
            }
            else if (keyPressed->code == sf::Keyboard::Key::Tab)
            {
                _spawnAABBMode = !_spawnAABBMode;
            }
        }

        if (const auto* mousePressed = event.getIf<sf::Event::MouseButtonPressed>())
        {
            if (mousePressed->button == sf::Mouse::Button::Left)
            {
                const auto worldPos = ScreenToWorld(mousePressed->position, _pixelsPerMeter);
                const bool spawnAABB = _spawnAABBMode
                                    || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift)
                                    || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift);

                if (spawnAABB)
                {
                    SpawnAABB(worldPos);
                }
                else
                {
                    SpawnCircle(worldPos);
                }
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
        RefreshCollisionPairs();
    }

    void Render(sf::RenderWindow& window) override
    {
        for (const auto& circlePtr : _circles)
        {
            const bool overlap = circlePtr->overlapCount > 0;

            const sf::Color fill = overlap ? sf::Color(220, 80, 80, 140)
                                           : sf::Color(80, 140, 220, 120);
            DrawCircle(window, *circlePtr, fill, sf::Color(255, 255, 255, 80), 1.0f);
        }

        for (const auto& aabbPtr : _aabbs)
        {
            const bool overlap = aabbPtr->overlapCount > 0;

            const sf::Color fill = overlap ? sf::Color(220, 120, 80, 140)
                                           : sf::Color(110, 180, 255, 120);
            DrawAABB(window, *aabbPtr, fill, sf::Color(255, 255, 255, 80), 1.0f);
        }

        const sf::Color probeColor = _probe.overlapCount > 0 ? sf::Color(255, 80, 80, 160)
                                                             : sf::Color(80, 240, 120, 160);
        DrawCircle(window, _probe, probeColor, sf::Color(255, 255, 255, 200), 2.0f);

        DrawCollisionDebug(window);
    }

    void BuildOverlay(std::vector<std::string>& lines) const override
    {
        lines.emplace_back("LEFT MOUSE: ADD CIRCLE");
        lines.emplace_back("SHIFT + LEFT MOUSE: ADD AABB");
        lines.emplace_back("TAB: TOGGLE SPAWN MODE");
        lines.emplace_back("RIGHT MOUSE DRAG: MOVE CIRCLE PROBE");
        lines.emplace_back("C: CLEAR SHAPES");
        lines.emplace_back("R: RESET DEMO");
        lines.emplace_back(std::string("SPAWN MODE: ") + (_spawnAABBMode ? "AABB" : "CIRCLE"));
        lines.emplace_back("CIRCLES: " + std::to_string(_circles.size()));
        lines.emplace_back("AABBS: " + std::to_string(_aabbs.size()));
        lines.emplace_back("OVERLAPS: " + std::to_string(CountOverlaps()));
        lines.emplace_back("POINT A: YELLOW, POINT B: CYAN");
    }

private:
    struct CircleBody
    {
        std::shared_ptr<Guch2D::CollisionBody> body;
        float radius = 0.0f;
        int overlapCount = 0;
    };

    struct AABBBody
    {
        std::shared_ptr<Guch2D::CollisionBody> body;
        Guch2D::Vect extent = {0.0f, 0.0f};
        int overlapCount = 0;
    };

    struct CollisionPair
    {
        std::weak_ptr<Guch2D::CollisionBody> bodyA;
        std::weak_ptr<Guch2D::CollisionBody> bodyB;
        Guch2D::CollisionPoints points;
    };

    using CircleBodyPtr = std::shared_ptr<CircleBody>;
    using AABBBodyPtr = std::shared_ptr<AABBBody>;

    CircleBody CreateCircle(const Guch2D::Vect& position, const float radius)
    {
        auto collider = std::make_shared<Guch2D::CircleCollider>();
        collider->SetRadius(radius);

        auto body = std::make_shared<Guch2D::DynamicRigidBody>(position, 2.0F);
        body->SetSimulatePhysics(false);
        body->SetCollider(collider);
        return {body, radius};
    }

    AABBBody CreateAABB(const Guch2D::Vect& position, const Guch2D::Vect& extent)
    {
        auto collider = std::make_shared<Guch2D::AABBCollider>();
        collider->SetExtent(extent);

        auto body = std::make_shared<Guch2D::DynamicRigidBody>(position, 2.0F);
        body->SetSimulatePhysics(false);
        body->SetCollider(collider);
        return {body, extent};
    }

    void SpawnCircle(const Guch2D::Vect& position)
    {
        auto circlePtr = std::make_shared<CircleBody>(CreateCircle(position, _spawnRadius));
        BindCollisionCallbacks(circlePtr->body, circlePtr->overlapCount);

        _circles.push_back(circlePtr);
        _world.AddObject(circlePtr->body);
    }

    void SpawnAABB(const Guch2D::Vect& position)
    {
        auto aabbPtr = std::make_shared<AABBBody>(CreateAABB(position, _spawnAABBExtent));
        BindCollisionCallbacks(aabbPtr->body, aabbPtr->overlapCount);

        _aabbs.push_back(aabbPtr);
        _world.AddObject(aabbPtr->body);
    }

    void BindCollisionCallbacks(const std::shared_ptr<Guch2D::CollisionBody>& body,
                                int& overlapCount)
    {
        body->BindOnBeginOverlap([this, &overlapCount](const Guch2D::Collision& callback) {
            ++overlapCount;
            RegisterCollision(callback);
        });
        body->BindOnEndOverlap([this, &overlapCount](const Guch2D::Collision& callback) {
            overlapCount = std::max(0, overlapCount - 1);
            RemoveCollision(callback);
        });
    }

    void ResetWorldWithProbe()
    {
        _world = Guch2D::DynamicWorld();
        _world.AddObject(_probe.body);
        _world.AddSolver(std::make_shared<Guch2D::PositionSolver>());
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

    void DrawAABB(sf::RenderWindow& window,
                  const AABBBody& aabb,
                  const sf::Color fill,
                  const sf::Color outline,
                  const float outlineThickness) const
    {
        const sf::Vector2f halfExtentPixels = {
            aabb.extent.x * _pixelsPerMeter,
            aabb.extent.y * _pixelsPerMeter,
        };

        sf::RectangleShape shape(halfExtentPixels * 2.0f);
        shape.setOrigin(halfExtentPixels);
        shape.setPosition(WorldToScreen(aabb.body->GetColliderCenterWorld(), _pixelsPerMeter));
        shape.setFillColor(fill);
        shape.setOutlineColor(outline);
        shape.setOutlineThickness(outlineThickness);
        window.draw(shape);
    }

    void DrawCollisionDebug(sf::RenderWindow& window) const
    {
        if (_collisionPairs.empty())
        {
            return;
        }

        const sf::Color normalColor(255, 200, 40, 220);
        const sf::Color pointAColor(255, 240, 60, 255);
        const sf::Color pointBColor(70, 235, 255, 255);
        for (const auto& pair : _collisionPairs)
        {
            if (!pair.points.HasCollision)
            {
                continue;
            }

            DrawContactPoint(window, pair.points.ContactPoints.front().Position, pointAColor);
            DrawContactPoint(window, pair.points.ContactPoints.back().Position, pointBColor);
            DrawArrow(window,
                      pair.points.ContactPoints.front().Position,
                      pair.points.ContactPoints.back().Position
                          - pair.points.ContactPoints.front().Position,
                      normalColor);
        }
    }

    void DrawContactPoint(sf::RenderWindow& window,
                          const Guch2D::Vect& pointWorld,
                          const sf::Color color) const
    {
        sf::CircleShape point(CollisionPointRadiusPixels);
        point.setOrigin({CollisionPointRadiusPixels, CollisionPointRadiusPixels});
        point.setPosition(WorldToScreen(pointWorld, _pixelsPerMeter));
        point.setFillColor(color);
        point.setOutlineColor(sf::Color::Black);
        point.setOutlineThickness(1.0f);
        window.draw(point);
    }

    void DrawArrow(sf::RenderWindow& window,
                   const Guch2D::Vect& startWorld,
                   const Guch2D::Vect& direction,
                   const sf::Color color) const
    {
        const auto normal = Guch2D::VectNormalize(direction);
        if (!Guch2D::IsFinite(normal) || (normal.x == 0.0f && normal.y == 0.0f))
        {
            return;
        }

        const Guch2D::Vect tipWorld = startWorld + normal * CollisionNormalLengthMeters;
        const Guch2D::Vect perp = {-normal.y, normal.x};
        const Guch2D::Vect leftWorld = tipWorld - normal * CollisionNormalHeadLengthMeters
                                     + perp * CollisionNormalHeadWidthMeters;
        const Guch2D::Vect rightWorld = tipWorld - normal * CollisionNormalHeadLengthMeters
                                      - perp * CollisionNormalHeadWidthMeters;

        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0] = sf::Vertex(WorldToScreen(startWorld, _pixelsPerMeter), color);
        line[1] = sf::Vertex(WorldToScreen(tipWorld, _pixelsPerMeter), color);
        window.draw(line);

        sf::VertexArray head(sf::PrimitiveType::Lines, 4);
        head[0] = sf::Vertex(WorldToScreen(tipWorld, _pixelsPerMeter), color);
        head[1] = sf::Vertex(WorldToScreen(leftWorld, _pixelsPerMeter), color);
        head[2] = sf::Vertex(WorldToScreen(tipWorld, _pixelsPerMeter), color);
        head[3] = sf::Vertex(WorldToScreen(rightWorld, _pixelsPerMeter), color);
        window.draw(head);
    }

    void RefreshCollisionPairs()
    {
        std::vector<CollisionPair> refreshedPairs;
        refreshedPairs.reserve(_collisionPairs.size());

        for (const auto& pair : _collisionPairs)
        {
            const auto bodyA = pair.bodyA.lock();
            const auto bodyB = pair.bodyB.lock();
            if (!bodyA || !bodyB)
            {
                continue;
            }

            auto points = CollisionWorldInspector::Check(bodyA, bodyB);
            if (!points.HasCollision)
            {
                continue;
            }

            refreshedPairs.push_back({bodyA, bodyB, points});
        }

        _collisionPairs = std::move(refreshedPairs);
    }

    void RegisterCollision(const Guch2D::Collision& collision)
    {
        if (!collision.Points.HasCollision)
        {
            return;
        }

        auto it = std::find_if(
            _collisionPairs.begin(),
            _collisionPairs.end(),
            [&](const CollisionPair& pair) { return IsSamePair(collision, pair); });
        if (it == _collisionPairs.end())
        {
            _collisionPairs.push_back({collision.BodyA, collision.BodyB, collision.Points});
            return;
        }

        it->points = collision.Points;
    }

    void RemoveCollision(const Guch2D::Collision& collision)
    {
        if (IsStillOverlapping(collision))
        {
            return;
        }

        std::erase_if(_collisionPairs,
                      [&](const CollisionPair& pair) { return IsSamePair(collision, pair); });
    }

    [[nodiscard]] bool IsStillOverlapping(const Guch2D::Collision& collision) const
    {
        const auto bodyA = collision.BodyA.lock();
        const auto bodyB = collision.BodyB.lock();
        if (!bodyA || !bodyB)
        {
            return false;
        }

        return CollisionWorldInspector::Check(bodyA, bodyB).HasCollision;
    }

    [[nodiscard]] static bool IsSamePair(const Guch2D::Collision& collision,
                                         const CollisionPair& pair)
    {
        const auto bodyA = collision.BodyA.lock();
        const auto bodyB = collision.BodyB.lock();
        const auto pairA = pair.bodyA.lock();
        const auto pairB = pair.bodyB.lock();

        if (!bodyA || !bodyB || !pairA || !pairB)
        {
            return false;
        }

        return (bodyA == pairA && bodyB == pairB) || (bodyA == pairB && bodyB == pairA);
    }

    std::size_t CountOverlaps() const
    {
        std::size_t count = 0;
        for (const auto& circlePtr : _circles)
        {
            if (circlePtr->overlapCount > 0)
            {
                ++count;
            }
        }

        for (const auto& aabbPtr : _aabbs)
        {
            if (aabbPtr->overlapCount > 0)
            {
                ++count;
            }
        }

        if (_probe.overlapCount > 0)
        {
            ++count;
        }

        return count;
    }

    float _pixelsPerMeter = PixelsPerMeter;
    std::vector<CircleBodyPtr> _circles;
    std::vector<AABBBodyPtr> _aabbs;
    std::vector<CollisionPair> _collisionPairs;
    CircleBody _probe;
    Guch2D::DynamicWorld _world;
    bool _draggingProbe = false;
    bool _spawnAABBMode = false;
    float _spawnRadius = 0.25f;
    float _probeRadius = 0.35f;
    Guch2D::Vect _spawnAABBExtent = {0.3f, 0.2f};
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
    demos.Add(std::make_unique<BoundedDynamicsDemo>(PixelsPerMeter));
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
