#include "stdafx.h"
#include "Collision/Collision.hpp"
#include "Dynamics/RigidBodyManager.hpp"
#include "Collision/SatCollision.hpp"
#include "Math.hpp"

std::unordered_set<RigidBody*> Collision::m_bodiesToUpdate;
std::vector<CollisionManifold> Collision::m_collidingBodies;

std::vector<sf::RectangleShape> Collision::m_points;

DynamicTree Collision::m_Tree;

void Collision::Update()
{
	m_points.clear();


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
			if (!CheckCollision(b1, b2, mtv) || DoesCollisionExist(b1,b2))
				continue;
			CollisionManifold manifold;
			manifold.A = b1;
			manifold.B = b2;
			manifold.mtv = mtv;

			manifold.FindContactPoints();
			m_collidingBodies.push_back(manifold);
		}
	}

	for (auto& c : m_collidingBodies) {
		ResolveCollisionWithFriction(c);
		if (c.count > 0) {
			sf::RectangleShape s{ {1.f,1.f}};
			s.setFillColor(sf::Color::Red);
			s.setOrigin(s.getSize() / 2.f);
			s.setPosition(c.cp1);
			m_points.push_back(s);
			if (c.count > 1) {
				s.setPosition(c.cp2);
				m_points.push_back(s);
			}
		}
	}
	m_collidingBodies.clear();

	for (auto& b : m_bodiesToUpdate)
		m_Tree.Update(b->m_nodeIndex);

	m_bodiesToUpdate.clear();
}

void Collision::Render(sf::RenderWindow& window)
{
	//m_Tree.Render(window);
	//for (auto& p : m_points)
	//	window.draw(p);
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
		if (!b2->m_isStatic) {
			b1->Move(-mtv / 2.f);
			b2->Move(-mtv/2.f);
		}
	}

	return good;
}

void Collision::ResolveCollision(CollisionManifold& manifold)
{
	RigidBody* b1 = manifold.A;
	RigidBody* b2 = manifold.B;
	sf::Vector2f mtv = manifold.mtv;
	sf::Vector2f normal = SATCollision::Instance.Normalize(mtv);
	sf::Vector2f relativeVelocity = b2->m_linearVelocity - b1->m_linearVelocity;

	if (dotProduct(-relativeVelocity, normal) > 0.f)
		return;

	float e = std::min(b1->m_data.Restitution, b2->m_data.Restitution);
	
	float j = -(1.f + e) * SATCollision::Instance.DotProduct(relativeVelocity, normal);
	j /= b1->m_data.InvMass + b2->m_data.InvMass;

	sf::Vector2f impulse = j * normal;

	b1->m_linearVelocity -= impulse * b1->m_data.InvMass;
	if (!b2->m_isStatic)
		b2->m_linearVelocity += impulse * b2->m_data.InvMass;
}

void Collision::ResolveCollisionWithRotation(CollisionManifold& c, float* jList, int jListCount)
{
	sf::Vector2f normal = SATCollision::Instance.Normalize(-c.mtv/2.f);
	float e = std::min(c.A->m_data.Restitution, c.B->m_data.Restitution);

	sf::Vector2f contactList[2]{c.cp1,c.cp2};
	sf::Vector2f raList[2]{};
	sf::Vector2f rbList[2]{};
	sf::Vector2f impulseList[2]{};

	for (int i = 0; i < c.count ; i++)
	{
		sf::Vector2f ra = contactList[i] - c.A->m_position;
		sf::Vector2f rb = contactList[i] - c.B->m_position;

		raList[i] = ra;
		rbList[i] = rb;

		sf::Vector2f  raPerp(-ra.y, ra.x);
		sf::Vector2f  rbPerp(-rb.y, rb.x);

		sf::Vector2f angularLinearVelocityA = raPerp * c.A->m_rotationalVelocity;
		sf::Vector2f angularLinearVelocityB = rbPerp * c.B->m_rotationalVelocity;

		sf::Vector2f relativeVelocity =
			(c.B->GetVelocity() + angularLinearVelocityB) -
			(c.A->GetVelocity() + angularLinearVelocityA);

		float contactVelocityMag = dotProduct(relativeVelocity, normal);

		if (contactVelocityMag > 0.f && !c.B->m_isStatic)
		{
			continue;
		}

		float raPerpDotN = dotProduct(raPerp, normal);
		float rbPerpDotN = dotProduct(rbPerp, normal);

		float denom = c.A->m_data.InvMass + c.B->m_data.InvMass +
			(raPerpDotN * raPerpDotN) * c.A->m_data.InvInertia +
			(rbPerpDotN * rbPerpDotN) * c.B->m_data.InvInertia;

		float j = -(1.f + e) * contactVelocityMag;
		j /= denom;
		j /= (float)c.count;

		if (i < jListCount)
			jList[i] = j;

		sf::Vector2f impulse = j * normal;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < c.count; i++)
	{
		c.A->m_linearVelocity += -impulseList[i] * c.A->m_data.InvMass;
		c.A->m_rotationalVelocity += -Cross(raList[i], impulseList[i]) * c.A->m_data.InvInertia;
		if (!c.B->m_isStatic) {
			c.B->m_linearVelocity += impulseList[i] * c.B->m_data.InvMass;
			c.B->m_rotationalVelocity += Cross(rbList[i], impulseList[i]) * c.B->m_data.InvInertia;
		}
	}
}

void Collision::ResolveCollisionWithFriction(CollisionManifold& c)
{
	sf::Vector2f normal = SATCollision::Instance.Normalize(-c.mtv / 2.f);
	float e = std::min(c.A->m_data.Restitution, c.B->m_data.Restitution);

	sf::Vector2f contactList[2]{ c.cp1,c.cp2 };
	sf::Vector2f raList[2]{};
	sf::Vector2f rbList[2]{};
	sf::Vector2f impulseList[2]{};
	float jList[2]{};

	ResolveCollisionWithRotation(c, jList,2);

	for (int i = 0; i < c.count; i++)
	{
		sf::Vector2f ra = contactList[i] - c.A->m_position;
		sf::Vector2f rb = contactList[i] - c.B->m_position;

		raList[i] = ra;
		rbList[i] = rb;

		sf::Vector2f  raPerp(-ra.y, ra.x);
		sf::Vector2f  rbPerp(-rb.y, rb.x);

		sf::Vector2f angularLinearVelocityA = raPerp * c.A->m_rotationalVelocity;
		sf::Vector2f angularLinearVelocityB = rbPerp * c.B->m_rotationalVelocity;

		sf::Vector2f relativeVelocity =
			(c.B->GetVelocity() + angularLinearVelocityB) -
			(c.A->GetVelocity() + angularLinearVelocityA);

		sf::Vector2f tangent = relativeVelocity - dotProduct(relativeVelocity, normal) * normal;

		if (NearlyEqual(tangent, sf::Vector2f({ 0.f,0.f }))) //prop static body
			continue;

		tangent = SATCollision::Instance.Normalize(tangent);

		float raPerpDotT = dotProduct(raPerp, tangent);
		float rbPerpDotT = dotProduct(rbPerp, tangent);

		float denom = c.A->m_data.InvMass + c.B->m_data.InvMass +
			(raPerpDotT * raPerpDotT) * c.A->m_data.InvInertia +
			(rbPerpDotT * rbPerpDotT) * c.B->m_data.InvInertia;

		float contactVelocityMag = dotProduct(relativeVelocity, tangent);

		float jt = -contactVelocityMag;
		jt /= denom;
		jt /= (float)c.count;

		sf::Vector2f frictionImpulse{};
		if (abs(jt) <= jList[i] * 0.5f) //static friction
			frictionImpulse = jt * tangent;
		else
			frictionImpulse = -jList[i] * tangent * 0.5f; //dynamic friction


		impulseList[i] = frictionImpulse;
	}

	for (int i = 0; i < c.count; i++)
	{
		c.A->m_linearVelocity += -impulseList[i] * c.A->m_data.InvMass;
		c.A->m_rotationalVelocity += -Cross(raList[i], impulseList[i]) * c.A->m_data.InvInertia;
		if (!c.B->m_isStatic) {
			c.B->m_linearVelocity += impulseList[i] * c.B->m_data.InvMass;
			c.B->m_rotationalVelocity += Cross(rbList[i], impulseList[i]) * c.B->m_data.InvInertia;
		}
	}
}

bool Collision::DoesCollisionExist(RigidBody* b1, RigidBody* b2)
{
	for (auto& c : m_collidingBodies) {
		if((c.A == b1 && c.B == b2) || (c.A == b2 && c.B == b1))
			return true;
	}
	return false;
}