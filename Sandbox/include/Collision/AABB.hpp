#pragma once

#include "SFML/System/Vector2.hpp"

struct AABB {
	sf::Vector2f lowerBound{};
	sf::Vector2f upperBound{};

	bool contains(sf::Vector2f point);
	bool contains(AABB other);
	bool intersects(AABB other);
	float GetPerimeter();
	float GetArea();
};