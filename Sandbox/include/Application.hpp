#pragma once

#include "stdafx.h"
#include "Random.hpp"

#include "Collision/Collision.hpp"
#include "Dynamics/RigidBodyManager.hpp"

class Application
{
public:
	Application();
	~Application();

	void Run();
private:
	void AddBody();
	void Init();
	void InitWindow();

	virtual void Update();
	virtual void Render();
	virtual void PollEvents();

	sf::RenderWindow m_window;

	RigidBody* m_bodyToMove;

	sf::Clock m_dt;
	float m_timeAccumulator = 0.f;
	float m_physicsTimeStep = 0.005f;

	bool m_mouseHold = true;
};