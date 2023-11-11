#include "stdafx.h"
#include "RigidBodyManager.hpp"

#include "CircleShape.hpp"

std::unordered_map<int, RigidBody*>  RigidBodyManager::m_bodies;
int RigidBodyManager::m_count;

void RigidBodyManager::Init()
{
	m_count = 0;
}

void RigidBodyManager::AddBody(RigidBody* body)
{
	m_bodies[m_count] = body;
	body->m_id = m_count;
	m_count++;
}

void RigidBodyManager::DeleteBody(int index)
{
	std::swap(m_bodies[index], m_bodies[m_count]);
	delete m_bodies[m_count];
	m_count--;

	m_bodies[index]->m_id = index;
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
	for (auto& body : m_bodies)
		body.second->Update(t);
}

void RigidBodyManager::Render(sf::RenderWindow& window)
{
	for (auto& body : m_bodies)
		body.second->Render(window);
}

int RigidBodyManager::GetCount()
{
	return m_count;
}

std::unordered_map<int, RigidBody*>& RigidBodyManager::GetBodies()
{
	return m_bodies;
}
