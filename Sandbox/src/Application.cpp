#include "stdafx.h"
#include "Application.hpp"
/*
Here we create window, and do all of the technical stuff of the Application
*/
Application::Application()
{
	Init();

	RigidBodyManager::Init();
	RigidBodyData d;
	d.Density = 0.7f;
	d.Mass = 10.f;
	d.Restitution = 0.5f;

	sf::Vector2f pos = { 10.f,10.f };

	RigidBody* b = new RigidBody(pos,d,false,0.f,10.f, 10.f, BODY_SHAPE::Box);
	RigidBody* b1 = new RigidBody(pos*2.5f,d,false,10.f,10.f,10.f,BODY_SHAPE::Box);
	b->Rotate(45.f);
	//b1->Rotate(45.f);
	RigidBodyManager::AddBody(b);
	RigidBodyManager::AddBody(b1);

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
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
			RigidBodyManager::GetBodies().at(0)->Move({ 0.f,5.f * t });
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
			RigidBodyManager::GetBodies().at(0)->Move({ 0.f,-5.f * t });
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
			RigidBodyManager::GetBodies().at(0)->Move({ -5.f * t,0.f });
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
			RigidBodyManager::GetBodies().at(0)->Move({ 5.f * t,0.f });
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