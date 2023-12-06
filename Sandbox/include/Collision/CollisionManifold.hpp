#pragma once
#include "Dynamics/RigidBody.hpp"

struct CollisionManifold {
	RigidBody* A;
	RigidBody* B;

	sf::Vector2f mtv = {};

	sf::Vector2f cp1 = {};
	sf::Vector2f cp2 = {};

	uint16_t count = 0;

	void FindContactPoints();
private:
	void FindPolyPolyContactPoint();
	void FindPolyCircleContactPoint(bool polyFirst);
	void FindCircleCircleContactPoint();
};