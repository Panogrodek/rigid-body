#include "stdafx.h"
#include "RigidBodyManager.hpp"
#include "Collision/Collision.hpp"


std::unordered_map<int, RigidBody*>  RigidBodyManager::m_bodies{};
std::unordered_set<int>  RigidBodyManager::m_bodiesToRemove{};
std::unordered_set<RigidBody*>  RigidBodyManager::m_bodiesToAdd{};
int RigidBodyManager::m_count = 0;

void RigidBodyManager::Init()
{
	Destroy();
}

void RigidBodyManager::AddBody(RigidBody* body)
{
	m_bodiesToAdd.insert(body);
}

void RigidBodyManager::DeleteBody(int index)
{
	m_bodiesToRemove.insert(index);
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
	AddBodies();
	RemoveBodies();
	for (auto& body : m_bodies) {
		body.second->Step(t);
	}
	Collision::Update();
}

void RigidBodyManager::Render(sf::RenderWindow& window)
{
	for (auto& body : m_bodies)
		body.second->Render(window);

	//debug only
	//Collision::Render();
}

int RigidBodyManager::GetCount()
{
	return m_count;
}

std::unordered_map<int, RigidBody*>& RigidBodyManager::GetBodies()
{
	return m_bodies;
}

void RigidBodyManager::RemoveBodies()
{
	while (m_bodiesToRemove.size()) {
		int id = *m_bodiesToRemove.begin();
		m_bodiesToRemove.erase(m_bodiesToRemove.begin());
		if (id == -1)
			continue;
		Collision::RemoveBody(id);
		if (m_count > 1) {
			std::swap(m_bodies[id], m_bodies[m_count - 1]);
			auto& pos = m_bodiesToRemove.find(m_count - 1);
			if (pos != m_bodiesToRemove.end()) {
				m_bodiesToRemove.erase(pos);
				m_bodiesToRemove.insert(id);
			}
			delete m_bodies[m_count - 1];
		}
		else
			delete m_bodies[id];

		m_bodies.erase(m_count - 1);
		m_count--;

		if (id != m_count) {
			m_bodies[id]->m_id = id;

			Collision::Validate(m_bodies[id]);
		}
	}
	m_bodiesToRemove.clear();
}

void RigidBodyManager::AddBodies()
{
	for (auto& body : m_bodiesToAdd) {
		body->UpdateVertices();
		Collision::AddBody(body);
		m_bodies[m_count] = body;
		body->m_id = m_count;
		m_count++;
	}
	m_bodiesToAdd.clear();
}
