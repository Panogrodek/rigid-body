#pragma once
#include "RigidBody.hpp"

class CircleShape : public RigidBody{
public:
	CircleShape() {};
	CircleShape(sf::Vector2f position, float radius);

	void SetRadius(float radius);
	float GetRadius();
private:
	float m_radius = 0.f;
};