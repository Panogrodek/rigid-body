#include "stdafx.h"
#include "Application.hpp"
/*
Here we create window, and do all of the technical stuff of the Application
*/
Application::Application()
{
	Init();

	RigidBodyManager::Init();
	RigidBodyData d1;
	d1.Density = 0.7f;
	d1.Mass = 1.f;
	d1.Restitution = 1.50f;

	RigidBodyData d2;
	d2.Density = 0.7f;
	d2.Mass = 1.f;
	d2.Restitution = 1.00f;

	sf::Vector2f pos = { 10.f,10.f };
	sf::Vector2f pos2 = { -20.f,0.f };

	RigidBody* b = new RigidBody(pos ,d1,false,0.f,10.f, 10.f, BODY_SHAPE::Box);
	RigidBody* b2 = new RigidBody(pos + pos2*2.f,d1,true,10.f,10.f,10.f,BODY_SHAPE::Box);
	RigidBody* b3 = new RigidBody(pos - pos2,d1,false,10.f,10.f,10.f,BODY_SHAPE::Box);
	RigidBody* b4 = new RigidBody({0.f,-25.f}, d2, true, 10.f, 100.f, 10.f, BODY_SHAPE::Box);
	//b->Rotate(45.f);
	b4->Rotate(22.5f);
	RigidBodyManager::AddBody(b);
	RigidBodyManager::AddBody(b2);
	RigidBodyManager::AddBody(b3);
	RigidBodyManager::AddBody(b4);
	//RigidBodyManager::GetBodies().at(1)->AddImpulse({ 1000.f,0.f });

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
	mode.size = sf::Vector2u(1000, 1000); //TODO: na windows 11 to jest podwojone xd
	m_window.create(mode, "", sf::Style::Close);
	m_window.setFramerateLimit(60);

	m_window.setView(sf::View({ {0.f, 0.f}, {100.f, -100.f} }));
}

void Application::Update()
{
	float frameTime = m_dt.getElapsedTime().asSeconds();
	m_dt.restart();

	float t = 0.0f;
	m_timeAccumulator += frameTime;

	//first update the movement


	while (m_timeAccumulator >= m_physicsTimeStep)
	{
		m_timeAccumulator -= m_physicsTimeStep;
		t += m_physicsTimeStep;
		sf::Vector2f vecDir{0.f,0.f};
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
			vecDir += {0.f, 1.f};
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
			vecDir += {0.f, -1.f};
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
			vecDir += {-1.f, 0.f};
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
			vecDir += {1.f, 0.f};

		RigidBodyManager::GetBodies().at(0)->AddImpulse(vecDir * 5.5f);

		RigidBodyManager::Update(m_physicsTimeStep);
	}

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
				m_window.close();
			break;
		}
	}
}