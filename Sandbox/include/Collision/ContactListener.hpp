#pragma once

#include "Dynamics/RigidBodies.hpp"

class ContactListener {
public:
	static void Init();

	static void CheckForCollisions();
private:
	friend class CollisionResolver;
	static std::vector<std::pair<int, int>> m_CollisionPairs;
};