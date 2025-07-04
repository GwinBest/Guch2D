#include "Collisions/AABBCollider.hpp"
#include "Collisions/CircleCollider.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/DynamicWorld.hpp"
#include "Dynamics/StaticRigidBody.hpp"
#include "SFML/Graphics.hpp"

int main()
{
    auto object1 = std::make_shared<Guch2D::DynamicRigidBody>();
    object1->SetPosition({2.0F, 0.2F});
    object1->SetMass(2.0F);
    object1->SetCollider(std::make_shared<Guch2D::CircleCollider>(Guch2D::Vect(0.0F, 0.0F), 0.1F));

    auto ground = std::make_shared<Guch2D::StaticRigidBody>();
    ground->SetPosition({1.7F, 6.0F});
    ground->SetMass(std::numeric_limits<float>::infinity());
    ground->SetCollider(
        std::make_shared<Guch2D::AABBCollider>(Guch2D::Vect(0.0F, 0.0F), Guch2D::Vect(1.F, 1.F)));

    Guch2D::DynamicWorld world;
    world.AddObject(object1);
    world.AddObject(ground);

    auto window = sf::RenderWindow(sf::VideoMode({1280, 720}), "Guch2D Physics Engine");
    window.setFramerateLimit(144);

    const float scale = 100.0F;

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        window.clear(sf::Color::White);

        const float deltaTime = 1.0F / 60.0F;
        world.SetTimeStep(deltaTime);
        world.Step();
        // circle
        auto pos1 = object1->GetPosition();
        auto circleCollider = std::dynamic_pointer_cast<Guch2D::CircleCollider>(
            object1->GetCollider());
        if (circleCollider)
        {
            float radius = circleCollider->GetRadius() * scale;
            sf::CircleShape shape(radius);
            shape.setFillColor(sf::Color::Green);
            shape.setOrigin({radius, radius});
            shape.setPosition({pos1.x * scale, pos1.y * scale});
            window.draw(shape);

            sf::CircleShape colliderShape(radius);
            colliderShape.setFillColor(sf::Color::Transparent);
            colliderShape.setOutlineColor(sf::Color::Red);
            colliderShape.setOutlineThickness(2.0f);
            colliderShape.setOrigin({radius, radius});
            colliderShape.setPosition({pos1.x * scale, pos1.y * scale});
            window.draw(colliderShape);
        }

        // AABB
        auto pos2 = ground->GetPosition();
        auto pos2Collider = ground->GetColliderCenterWorld();
        auto aabbCollider = std::dynamic_pointer_cast<Guch2D::AABBCollider>(ground->GetCollider());
        if (aabbCollider)
        {
            Guch2D::Vect halfSize = aabbCollider->GetExtends();
            float width = halfSize.x * 2 * scale;
            float height = halfSize.y * 2 * scale;
            sf::RectangleShape rect({width, height});
            rect.setFillColor(sf::Color(0, 100, 200));
            rect.setOrigin({width / 2, height / 2});
            rect.setPosition({pos2.x * scale, pos2.y * scale});
            window.draw(rect);

            sf::RectangleShape colliderRect({width, height});
            colliderRect.setFillColor(sf::Color::Transparent);
            colliderRect.setOutlineColor(sf::Color::Blue);
            colliderRect.setOutlineThickness(2.0f);
            colliderRect.setOrigin({width / 2, height / 2});
            colliderRect.setPosition({pos2Collider.x * scale, pos2Collider.y * scale});
            window.draw(colliderRect);
        }

        window.display();
    }
}