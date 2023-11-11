#include "stdafx.h"
#include "ContactListener.hpp"
#include "Dynamics/RigidBodyManager.hpp"

std::vector<std::pair<int, int>> ContactListener::m_CollisionPairs;

void ContactListener::Init()
{
}

void ContactListener::CheckForCollisions()
{
	m_CollisionPairs.clear();
	//TODO: add BVH collision checking to test ONLY aabbs of the bodies
	int startIndex = 1;
	for (auto& b1 : RigidBodyManager::GetBodies()) {
		for (int i = startIndex; i < RigidBodyManager::GetCount(); i++) {
			auto& b2 = RigidBodyManager::GetBodies()[i];
			if(b1.second->GetAABB().intersects(b2->GetAABB()))
				m_CollisionPairs.push_back(std::make_pair(b1.first,i));
		}
		startIndex++;
	}
}
