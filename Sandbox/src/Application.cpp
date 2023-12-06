#include "stdafx.h"
#include "Application.hpp"
#include "Random.hpp"
/*
Here we create window, and do all of the technical stuff of the Application
*/
Application::Application()
{
	Init();

	RigidBodyManager::Init();
	RigidBodyData d1;
	d1.Density = 0.7f;
	d1.Mass = 10.f;
	d1.Restitution = 1.20f;
	d1.Friction = 0.4f;

	RigidBodyData d2;
	d2.Density = 0.7f;
	d2.Mass = 10.1f;
	d2.Restitution = 0.00f;
	d2.Friction = 0.6f;

	sf::Vector2f pos = { 10.f,10.f };
	sf::Vector2f pos2 = { 0.f,15.f };

	//RigidBody* b = new RigidBody(pos ,d1,false,0.f,10.f, 10.f, BODY_SHAPE::Box);
	RigidBody* leverBase = new RigidBody({0.f,0.f}, d2, false, 10.f, 2.5f, 5.0f, BODY_SHAPE::Box);
	RigidBody* lever = new RigidBody({ 0.f,24.0f }, d2, false, 10.f, 30.f, 2.5f, BODY_SHAPE::Box);

	RigidBody* ball = new RigidBody({10.f - 0.01f,15.f}, d1, false, 5.f, 10.f, 10.f, BODY_SHAPE::Circle);
	RigidBody* ball2 = new RigidBody({-9.5f ,6.5f}, d2, false, 1.f, 10.f, 10.f, BODY_SHAPE::Circle);

	RigidBody* b6 = new RigidBody(pos + pos2*2.f, d1, false, 10.f, 10.f, 10.f, BODY_SHAPE::Box);
	RigidBody* platform = new RigidBody({0.f,-7.5f}, d2, true, 10.f, 80.f, 10.f, BODY_SHAPE::Box);
	//b2->Rotate(45.f);
	//b4->Rotate(12.25f);
	//RigidBodyManager::AddBody(b);
	RigidBodyManager::AddBody(platform);
	RigidBodyManager::AddBody(ball);
	RigidBodyManager::AddBody(ball2);

	RigidBodyManager::AddBody(leverBase);
	RigidBodyManager::AddBody(lever);
	//RigidBodyManager::AddBody(b5);
	//RigidBodyManager::AddBody(b6);

	//RigidBodyManager::GetBodies().at(1)->AddImpulse({ 0500.f,1000.f });

}

Application::~Application()
{
}

void Application::Run()
{
	while (m_window.isOpen()) {
		PollEvents();
		//if(m_window.hasFocus()) {
		Update();
		Render();
		//}
	}
}

void Application::AddBody()
{
	RigidBodyData d1;
	d1.Density = 0.7f;
	d1.Mass = 50.f;
	d1.Restitution = 0.50f;
	d1.Friction = 0.4f;

	sf::Vector2f Pos = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window));
	RigidBody* b4 = new RigidBody(Pos, d1, false, Rand(3.f, 9.f), Rand(3.f,9.f), Rand(3.f, 9.f), BODY_SHAPE((Rand(1, 1))));
	b4->color = sf::Color(int(Rand(0.f,255.f)), int(Rand(0.f, 255.f)), int(Rand(0.f, 255.f)),255);
	RigidBodyManager::AddBody(b4);
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
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && !m_mouseHold) {
			m_mouseHold = true;
			AddBody();
		}
		else if (!sf::Mouse::isButtonPressed(sf::Mouse::Left))
			m_mouseHold = false;

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

		RigidBodyManager::GetBodies().at(1)->AddImpulse(vecDir * 10.5f);

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