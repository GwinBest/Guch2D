#include <SFML/Graphics.hpp>
#include <iostream>

#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/DynamicWorld.hpp"
#include "Math/Vector.hpp"

void BasicDynamicsDemo()
{
    auto window = sf::RenderWindow(sf::VideoMode({1280, 720}), "Guch2D");
    window.setFramerateLimit(144);

    Guch2D::DynamicWorld dynamicWorld;

    std::vector<std::shared_ptr<Guch2D::DynamicRigidBody>> bodies;

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
            else if (event->is<sf::Event::MouseButtonPressed>())
            {
                if (const auto* mouseButton = event->getIf<sf::Event::MouseButtonPressed>();
                    mouseButton->button == sf::Mouse::Button::Left)
                {
                    auto body = std::make_shared<Guch2D::DynamicRigidBody>(
                        Guch2D::Vect(12.0F, 4.0F),
                        10.0F);

                    body->AddForce({-5200.0F, -1200.0F});

                    dynamicWorld.AddObject(body);
                    bodies.push_back(body);
                }
            }
        }

        dynamicWorld.Step();

        window.clear(sf::Color::Black);

        for (const auto& body : bodies)
        {
            constexpr float scale = 100.0F;
            const auto pos = body->GetPosition();

            sf::CircleShape shape(10.f);
            shape.setFillColor(sf::Color::Green);
            shape.setOrigin({10.f, 10.f});
            shape.setPosition({pos.x * scale, pos.y * scale});

            window.draw(shape);
        }

        window.display();
    }
}

int main()
{
    BasicDynamicsDemo();

    return 0;
}
