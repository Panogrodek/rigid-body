#pragma once
#include "SFML/System/Vector2.hpp"
#include "SFML/Graphics/ConvexShape.hpp"

inline sf::Vector2f subtract(sf::Vector2f a, sf::Vector2f b) { a.x -= b.x; a.y -= b.y; return a; }
inline sf::Vector2f negate(sf::Vector2f v) { v.x = -v.x; v.y = -v.y; return v; }
inline sf::Vector2f perpendicular(sf::Vector2f v) { sf::Vector2f p = { v.y, -v.x }; return p; }
inline float dotProduct(sf::Vector2f a, sf::Vector2f b) { return a.x * b.x + a.y * b.y; }
inline float lengthSquared(sf::Vector2f v) { return v.x * v.x + v.y * v.y; }

inline sf::Vector2f tripleProduct(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) {

    sf::Vector2f r;

    float ac = a.x * c.x + a.y * c.y; // perform a.dot(c)
    float bc = b.x * c.x + b.y * c.y; // perform b.dot(c)

    // perform b * a.dot(c) - a * b.dot(c)
    r.x = b.x * ac - a.x * bc;
    r.y = b.y * ac - a.y * bc;
    return r;
}

inline sf::Vector2f averagePoint(const sf::Vector2f* vertices, size_t count) {
    sf::Vector2f  avg = { 0.f, 0.f };
    for (size_t i = 0; i < count; i++) {
        avg.x += vertices[i].x;
        avg.y += vertices[i].y;
    }
    avg.x /= count;
    avg.y /= count;
    return avg;
}

inline float PointLineDistance(sf::Vector2f a, sf::Vector2f b, sf::Vector2f p, sf::Vector2f& out) {
    sf::Vector2f ab = b - a;
    sf::Vector2f ap = p - a;

    float proj = dotProduct(ap, ab);
    float lengthSquared = ab.x * ab.x + ab.y * ab.y;
    float d = proj / lengthSquared;

    if (d <= 0.f)
        out = a;
    else if (d >= 1.f)
        out = b;
    else
        out = a + ab * d;

    sf::Vector2f pOut = p - out;

    return sqrt(pOut.x * pOut.x + pOut.y * pOut.y);
}

inline bool NearlyEqual(float a, float b){
    float result = b - a;

    if (abs(result) < 0.005)
        return true;
    return false;
}

inline bool NearlyEqual(sf::Vector2f a, sf::Vector2f b) {
    return NearlyEqual(a.x, b.x) && NearlyEqual(a.y, b.y);
}

inline float Cross(sf::Vector2f a, sf::Vector2f b) {
    return a.x * b.y - a.y * b.x;
}

inline sf::Vector2f Normalize(sf::Vector2f v) {
    return v /= v.length();
}