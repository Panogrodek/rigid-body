#include "stdafx.h"
#include "CircleShape.hpp"

#include "Collision/Collision.hpp"

CircleShape::CircleShape(sf::Vector2f position, float radius)
{
	m_shape = BODY_SHAPE::Circle;

	m_position = position;
	m_radius = radius;

	m_renderBody.setSize({m_radius * 2.f, m_radius * 2.f});

	sf::Vector2f halfSize = m_renderBody.getSize() / 2.f;
	m_renderBody.setOrigin(halfSize);


	m_aabb.lowerBound = { m_position - halfSize };
	m_aabb.upperBound = { m_position + halfSize };

	m_renderBody.setPosition(position);
	static sf::Texture texture;
	if(texture.getSize().x == 0.f)
		texture.loadFromFile("res/circle.png");
	m_renderBody.setTexture(&texture);
}

void CircleShape::SetRadius(float radius)
{
	m_radius = radius;
}

float CircleShape::GetRadius()
{
	return m_radius;
}
