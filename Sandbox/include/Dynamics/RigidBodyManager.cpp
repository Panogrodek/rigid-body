#include "stdafx.h"
#include "RigidBodyManager.hpp"
#include "Collision/Collision.hpp"

std::unordered_map<int, RigidBody*>  RigidBodyManager::m_bodies;
int RigidBodyManager::m_count;

void RigidBodyManager::Init()
{
	m_count = 0;
}

void RigidBodyManager::AddBody(RigidBody* body)
{
	Collision::AddBody(body);
	m_bodies[m_count] = body;
	body->m_id = m_count;
	m_count++;
}

void RigidBodyManager::DeleteBody(int index)
{
	Collision::RemoveBody(index);
	std::swap(m_bodies[index], m_bodies[m_count]);

	delete m_bodies[m_count];
	m_bodies.erase(m_count - 1);
	m_count--;

	if (index != m_count) {
		m_bodies[index]->m_id = index;
	}
}

void RigidBodyManager::DeleteBody(RigidBody* body)
{
	DeleteBody(body->m_id);
}

void RigidBodyManager::Destroy()
{
	for (auto& body : m_bodies)
		delete body.second;

	m_bodies.clear();
	m_count = 0;
}

void RigidBodyManager::Update(float t)
{
	for (auto& body : m_bodies) {
		body.second->Step(t);
		std::cout << body.second->GetVelocity().y << "\n";
	}

	Collision::Update();

	//broad phase collision
	//ContactListener::CheckForCollisions();

	//narrow phase collision
	//CollisionResolver::ResolveCollisions();
}

void RigidBodyManager::Render(sf::RenderWindow& window)
{
	for (auto& body : m_bodies)
		body.second->Render(window);

	Collision::Render(window);
}

int RigidBodyManager::GetCount()
{
	return m_count;
}

std::unordered_map<int, RigidBody*>& RigidBodyManager::GetBodies()
{
	return m_bodies;
}
