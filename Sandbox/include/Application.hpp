#pragma once

#include "stdafx.h"
#include "Random.hpp"

#include "Dynamics/RigidBodies.hpp"

#include "Collision/ContactListener.hpp"
#include "Collision/CollisionResolver.hpp"
#include "Dynamics/RigidBodyManager.hpp"

class Application
{
public:
	Application();
	~Application();

	void Run();
private:

	void Init();
	void InitWindow();

	virtual void Update();
	virtual void Render();
	virtual void PollEvents();

	sf::RenderWindow m_window;

	sf::Clock m_dt;
	float m_timeAccumulator = 0.f;
	float m_physicsTimeStep = 0.01f;
};