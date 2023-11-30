#include "stdafx.h"
#include "Collision/Collision.hpp"
#include "Dynamics/RigidBodyManager.hpp"
#include "Collision/SatCollision.hpp"

std::unordered_set<RigidBody*> Collision::m_bodiesToUpdate;

void Collision::Update()
{
	for (auto& b1 : m_bodiesToUpdate) {
		for (auto& [i,b2] : RigidBodyManager::m_bodies) {
			if (b1 == b2)
				continue;
			sf::Vector2f mtv{};
			CheckCollision(b1, b2, mtv);

			b1->Move(mtv);
			b2->Move(-mtv);
		}
	}

	m_bodiesToUpdate.clear();
}

bool Collision::CheckCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv)
{
	return SATCollision::Instance.SatCollision(*b1,*b2,mtv);
}
