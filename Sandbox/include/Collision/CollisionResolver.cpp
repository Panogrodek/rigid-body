#include "stdafx.h"
#include "CollisionResolver.hpp"

#include "Dynamics/RigidBodyManager.hpp"
#include "Collision/ContactListener.hpp"

void CollisionResolver::Init()
{
}

void CollisionResolver::ResolveCollisions()
{
	for (auto& [b1, b2] : ContactListener::m_CollisionPairs) {
		RigidBody* body1 = RigidBodyManager::GetBodies()[b1];
		RigidBody* body2 = RigidBodyManager::GetBodies()[b2];

		
	}
}
