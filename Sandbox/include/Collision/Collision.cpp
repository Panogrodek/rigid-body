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
		if (b1->m_isStatic) //this ignores the input problem, propably not good
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
	body->GetTransformedVertices(); //FUCK ME
	m_Tree.Insert(body);
}

void Collision::RemoveBody(int index)
{
	//TODO: debug
	m_Tree.RemoveLeafNode(RigidBodyManager::m_bodies[index]->m_nodeIndex);
}

bool Collision::CheckCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv)
{
	bool good = SATCollision::Instance.SatCollision(*b1, *b2, mtv);
	if (good) {
		b1->Move(mtv);
		if (!b2->m_isStatic)
			b2->Move(-mtv);
	}

	return good;
}

void Collision::ResolveCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv)
{
	sf::Vector2f normal = SATCollision::Instance.Normalize(mtv);
	sf::Vector2f relativeVelocity = b2->m_linearVelocity - b1->m_linearVelocity;

	float e = std::min(b1->m_data.Restitution, b2->m_data.Restitution);
	
	float j = -(1.f + e) * SATCollision::Instance.DotProduct(relativeVelocity, normal);
	j /= b1->m_data.InvMass + b2->m_data.InvMass;

	sf::Vector2f impulse = j * normal;

	b1->m_linearVelocity -= impulse * b1->m_data.InvMass;
	if (!b2->m_isStatic)
		b2->m_linearVelocity += impulse * b2->m_data.InvMass;
}
