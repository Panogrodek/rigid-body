#pragma once

#include "stdafx.h"
#include "Random.hpp"
#include "GJK.hpp"

#include "Dynamics/CircleShape.hpp"

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
	CircleShape circle;
};