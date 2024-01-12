#include "stdafx.h"
#include "AABB.hpp"

bool AABB::contains(sf::Vector2f point)
{
	if (point.x < lowerBound.x || point.x > upperBound.x)
		return false;
	if (point.y < lowerBound.y || point.y > upperBound.y)
		return false;
	return true;
}

bool AABB::contains(AABB other)
{
	return contains(other.lowerBound) && contains(other.upperBound);
}

bool AABB::intersects(AABB other)
{
	return (lowerBound.x <= other.upperBound.x && upperBound.x >= other.lowerBound.x) &&
		(lowerBound.y <= other.upperBound.y && upperBound.y >= other.lowerBound.y);
}

float AABB::GetPerimeter()
{
	sf::Vector2f d = upperBound - lowerBound;
	return d.x * d.y;
}

float AABB::GetArea()
{
	sf::Vector2f d = upperBound - lowerBound;

	return 2.0f * d.x * d.y;
}