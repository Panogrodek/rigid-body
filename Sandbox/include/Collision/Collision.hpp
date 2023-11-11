#pragma once
#include "Dynamics/RigidBodies.hpp"
class Collision {
public:
	static bool CheckCollision(CircleShape* b1, CircleShape* b2);
};