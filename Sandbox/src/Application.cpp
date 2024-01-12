#include "stdafx.h"
#include "Application.hpp"
#include "Random.hpp"

Application::Application()
{
	Init();

	RigidBodyManager::Init();

	//some random shapes initialized below
	RigidBodyPhysicsData d1;
	d1.Mass = 10.f;
	d1.Restitution = 0.6f;

	RigidBodyPhysicsData d2;
	d2.Mass = 10.1f;
	d2.Restitution = 0.00f;

	sf::Vector2f pos = { 10.f,10.f };
	sf::Vector2f pos2 = { 0.f,15.f };

	//RigidBody* b = new RigidBody(pos ,d1,false,0.f,10.f, 10.f, BODY_SHAPE::Box);
	RigidBody* leverBase =	new RigidBody({0.f,0.f},	d2,	10.f, 2.5f, 5.0f, false, BODY_SHAPE::Box);
	RigidBody* lever =		new RigidBody({ 0.f,24.0f },d2, 10.f, 30.f, 2.5f, false, BODY_SHAPE::Box);

	RigidBody* ball =		new RigidBody({10.f - 0.01f,15.f},	d1, 5.f, 10.f, 10.f, false, BODY_SHAPE::Circle);
	RigidBody* ball2 =		new RigidBody({ -9.5f ,6.5f },		d2, 1.f, 10.f, 10.f, false, BODY_SHAPE::Circle);

	RigidBody* box =		new RigidBody(pos + pos2*2.f,d1, 10.f, 10.f, 10.f,false, BODY_SHAPE::Box);
	RigidBody* platform =	new RigidBody({0.f,-7.5f},	 d2, 10.f, 80.f, 10.f, true, BODY_SHAPE::Box);
	RigidBody* platform2 =	new RigidBody({0.f,-7.5f},	 d2, 10.f, 80.f, 10.f, true, BODY_SHAPE::Box);
	platform2->Rotate(sf::degrees(90.f));

	m_bodyToMove = box;

	RigidBodyManager::AddBody(leverBase);
	RigidBodyManager::AddBody(lever);

	RigidBodyManager::AddBody(ball);
	RigidBodyManager::AddBody(ball2);

	RigidBodyManager::AddBody(box);
	RigidBodyManager::AddBody(platform);
	//RigidBodyManager::AddBody(platform2);

	RigidBodyManager::Update(0.f);
	for (auto& [index, body] : RigidBodyManager::GetBodies()) {
		body->AddLinearForce({ 0.f,-9.81f }); //Applying gravity
	}
}

Application::~Application()
{
}

void Application::Run()
{
	while (m_window.isOpen()) {
		PollEvents();
		Update();
		Render();
	}
}

void Application::AddBody() //this is an example how to create rigid bodies
{
	RigidBodyPhysicsData data; //first, we create physics data
	data.Mass = 50.f; //we assign some mass
	data.Restitution = 0.50f; //we assign some restitution (bounciness of the body)
	data.Inertia = 0.10f; //we assign some inertia (resistance to rotation)

	sf::Vector2f Pos = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window)); //position relative to mouse pos
	RigidBody* body = new RigidBody(
		Pos, //position of the body
		data, //physics data
		Rand(3.f, 9.f), //radius (only used with circle rigid bodies)
		Rand(3.f,9.f), //width (only used with rectangle shaped rigid bodies)
		Rand(3.f, 9.f), //height (only used with rectangle shaped rigid bodies)
		false, //is the body static i.e. can it be moved with forces?
		BODY_SHAPE((Rand(0, 1)) //is the body a circle or a box?
	));

	body->AddLinearForce({ 0.f,-9.81f }); //Applying gravity
	
	body->GetRenderBody().setFillColor(
		sf::Color(int(Rand(0.f, 255.f)), int(Rand(0.f, 255.f)), int(Rand(0.f, 255.f)), 255));
	RigidBodyManager::AddBody(body); //we also need to remember to add the rigid body to the manager
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

	//User input update
	
	//example body movement
	sf::Vector2f vecDir{ 0.f,0.f };
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
		vecDir += {0.f, 1.f};
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
		vecDir += {0.f, -1.f};
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
		vecDir += {-1.f, 0.f};
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
		vecDir += {1.f, 0.f};
	m_bodyToMove->AddLinearImpulse(vecDir * 100.0f);

	//Adding more bodies
	if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && !m_mouseHold) {
		m_mouseHold = true;
		AddBody();
	}
	else if (!sf::Mouse::isButtonPressed(sf::Mouse::Left))
		m_mouseHold = false;

	//physics update
	while (m_timeAccumulator >= m_physicsTimeStep)
	{
		m_timeAccumulator -= m_physicsTimeStep;
		t += m_physicsTimeStep;

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