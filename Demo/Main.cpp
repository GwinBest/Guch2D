#include <SFML/Graphics.hpp>

#include "Dynamics/DynamicWorld.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Math/Vector.hpp"

int main()
{
    auto window = sf::RenderWindow(sf::VideoMode({1280, 720}), "Guch2D");
    window.setFramerateLimit(144);

    Guch2D::DynamicWorld dynamicWorld;
    const auto dynamicRigidBody = std::make_shared<
        Guch2D::DynamicRigidBody>(
        Guch2D::Vect(1.0F, 1.0F),
        10.0F);
    dynamicWorld.AddObject(dynamicRigidBody);

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        dynamicWorld.Step();

        window.clear(sf::Color::Black);

        const auto position = dynamicRigidBody->GetPosition();
        constexpr float scale = 100.0f;

        sf::CircleShape shape(10.0f);
        shape.setFillColor(sf::Color::Green);
        shape.setOrigin({10.0f, 10.0f});
        shape.setPosition({position.x * scale, position.y * scale});

        window.draw(shape);

        window.display();
    }

    return 0;
}
