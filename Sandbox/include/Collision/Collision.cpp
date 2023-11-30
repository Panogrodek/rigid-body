#include "stdafx.h"
#include "Collision/Collision.hpp"
#include "Dynamics/RigidBodyManager.hpp"
#include "Collision/SatCollision.hpp"

std::unordered_set<RigidBody*> Collision::m_bodiesToUpdate;
DynamicTree Collision::m_Tree;

void Collision::Update()
{
	for (auto& b : m_bodiesToUpdate)
		m_Tree.Update(b->m_nodeIndex);

	m_bodiesToUpdate.clear();

	for (auto& [i, b1] : RigidBodyManager::m_bodies) {
		if (b1->m_isStatic)
			continue;

		std::vector<RigidBody*> collisions;
		collisions = m_Tree.GetCollisions(b1);

		for (auto& b2 : collisions) {
			sf::Vector2f mtv{};
			if (CheckCollision(b1, b2, mtv))
				ResolveCollision(b1, b2, mtv);
		}
	}

	for (auto& b : m_bodiesToUpdate)
		m_Tree.Update(b->m_nodeIndex);

	m_bodiesToUpdate.clear();
}

void Collision::Render(sf::RenderWindow& window)
{
	m_Tree.Render(window);
}

void Collision::AddBody(RigidBody* body)
{
	m_Tree.Insert(body);
}

void Collision::RemoveBody(int index)
{
	//TODO: debug
	m_Tree.RemoveLeafNode(RigidBodyManager::m_bodies[index]->m_nodeIndex);
}

bool Collision::CheckCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv)
{
	return SATCollision::Instance.SatCollision(*b1,*b2,mtv);
}

void Collision::ResolveCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv)
{
		b1->Move(mtv);
		if (!b2->m_isStatic) {
			b2->Move(-mtv);
		}
}
