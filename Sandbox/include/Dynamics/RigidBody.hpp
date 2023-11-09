#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/RenderWindow.hpp>

constexpr int INVALID_ID = -1;

class RigidBody {

	void Move(sf::Vector2f moveVec);
	void Scale(float scalar);
	void Rotate(float angle);

	void Render(sf::RenderWindow& window);

private:
	float angle;
	int m_VertexCount = 0;
};