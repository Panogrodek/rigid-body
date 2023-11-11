#include "stdafx.h"
#include "Application.hpp"
/*
Here we create window, and do all of the technical stuff of the Application
*/
Application::Application()
{
	Init();

	RigidBodyManager::Init();
	RigidBodyManager::AddBody(new CircleShape({ 0.f,0.f }, 50.f));
	RigidBodyManager::AddBody(new CircleShape({ -50.f,0.f }, 20.f));
}

Application::~Application()
{
}

void Application::Run()
{
	while (m_window.isOpen()) {
		PollEvents();
		/*	if(m_window.hasFocus()) {*/
		Update();
		Render();
		/*	}*/
	}
}

void Application::Init()
{
	InitWindow();
	m_dt.restart();
}

void Application::InitWindow()
{
	sf::VideoMode mode;
	mode.size = sf::Vector2u(1000, 1000);
	m_window.create(mode, "", sf::Style::Close);
	m_window.setFramerateLimit(60);

	m_window.setView(sf::View({ {0.f, 0.f}, {1000.f, -1000.f} }));
}

void Application::Update()
{
	float frameTime = m_dt.getElapsedTime().asSeconds();
	m_dt.restart();

	double t = 0.0;
	m_timeAccumulator += frameTime;

	//first update the movement

	while (m_timeAccumulator >= m_physicsTimeStep)
	{
		RigidBodyManager::Update(m_physicsTimeStep);
		m_timeAccumulator -= m_physicsTimeStep;
		t += m_physicsTimeStep;
	}

	//broad phase collision
	ContactListener::CheckForCollisions();

	//narrow phase collision
	CollisionResolver::ResolveCollisions();
}

void Application::Render()
{
	m_window.clear();

	RigidBodyManager::Render(m_window);

	m_window.display();
}

void Application::PollEvents()
{
	sf::Event ev;
	while (m_window.pollEvent(ev)) {
		switch (ev.type)
		{
		case sf::Event::Closed:
			m_window.close();
			break;
		case sf::Event::KeyPressed:
			if (ev.key.code == sf::Keyboard::Escape)
				m_window.close();;
			break;
		}
	}
}