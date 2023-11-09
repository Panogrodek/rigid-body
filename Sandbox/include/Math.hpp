#pragma once
#include "SFML/System/Vector2.hpp"
#include "SFML/Graphics/ConvexShape.hpp"

sf::Vector2f subtract(sf::Vector2f a, sf::Vector2f b) { a.x -= b.x; a.y -= b.y; return a; }
sf::Vector2f negate(sf::Vector2f v) { v.x = -v.x; v.y = -v.y; return v; }
sf::Vector2f perpendicular(sf::Vector2f v) { sf::Vector2f p = { v.y, -v.x }; return p; }
float dotProduct(sf::Vector2f a, sf::Vector2f b) { return a.x * b.x + a.y * b.y; }
float lengthSquared(sf::Vector2f v) { return v.x * v.x + v.y * v.y; }

sf::Vector2f tripleProduct(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) {

    sf::Vector2f r;

    float ac = a.x * c.x + a.y * c.y; // perform a.dot(c)
    float bc = b.x * c.x + b.y * c.y; // perform b.dot(c)

    // perform b * a.dot(c) - a * b.dot(c)
    r.x = b.x * ac - a.x * bc;
    r.y = b.y * ac - a.y * bc;
    return r;
}

sf::Vector2f averagePoint(const sf::Vector2f* vertices, size_t count) {
    sf::Vector2f  avg = { 0.f, 0.f };
    for (size_t i = 0; i < count; i++) {
        avg.x += vertices[i].x;
        avg.y += vertices[i].y;
    }
    avg.x /= count;
    avg.y /= count;
    return avg;
}