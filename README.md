# Guch2D

[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue)]()
[![CMake](https://img.shields.io/badge/CMake-enabled-brightgreen)]()
[![License](https://img.shields.io/badge/License-MIT-lightgrey)]()

Guch2D is a compact and high-performance 2D physics engine written in C++23. It focuses on easy integration, a clear API, and a balance between performance and physical accuracy.

Contents
- About
- Key features
- Requirements
- Quick usage example

## About
Guch2D implements core 2D physics components: bodies, collision detection (AABB/polygons/circles), motion integration, and collision resolution. The codebase is intentionally compact and readable to make it easy to extend and learn from.

## Key features
- Clean, modern C++23 API.
- Basic primitives: Circle, AABB, Polygon.
- Collision detection and contact resolution.
- Configurable integrators (Euler, semi-implicit Euler, etc.).
- Minimal external dependencies — CMake-based build.
- Small footprint and focus on run-time performance.
  
## Requirements
- CMake 3.19 or newer
- A C++23-capable compiler
- Git, used by CMake `FetchContent`
- Internet access on the first configure, so CMake can fetch dependencies

Fetched dependencies:

| Dependency | Used For |
| --- | --- |
| `glm` | Math support |
| `SFML 3.0.2` | Demo application |
| `GoogleTest 1.17.0` | Unit tests |

## Quick usage example
A simplified example showing how to create a world, add a body, and step the simulation:

```cpp
#include "Collision/CircleCollider.hpp"
#include "Dynamics/DynamicRigidBody.hpp"
#include "Dynamics/DynamicWorld.hpp"
#include "Solver/VelocitySolver.hpp"

int main()
{
    // 1. Initialize the physics world and required solvers
    Guch2D::DynamicWorld world;
    world.AddSolver(std::make_shared<Guch2D::PositionSolver>());
    world.AddSolver(std::make_shared<Guch2D::VelocitySolver>());

    // 2. Create a circular dynamic body (position: {0, 5}, mass: 10 kg, radius: 0.5 m)
    auto collider = std::make_shared<Guch2D::CircleCollider>();
    collider->SetRadius(0.5f);
    auto body = std::make_shared<Guch2D::DynamicRigidBody>(Guch2D::Vect{0.0f, 5.0f}, 10.0f);
    body->SetCollider(collider);
    body->SetVelocity({2.0f, 0.0f}); // Set initial velocity

    world.AddObject(body);

    // 3. Step the physics simulation in the game loop
    world.Step();

    return 0;
}
