#include "stdafx.h"
#include "CircleShape.hpp"

CircleShape::CircleShape(sf::Vector2f position, float radius) :
	m_position(position), m_radius(radius)
{
	m_renderBody.setSize({m_radius * 2.f, m_radius * 2.f});
	m_renderBody.setOrigin(m_renderBody.getSize() / 2.f);

	m_renderBody.setPosition(position);
	static sf::Texture texture;
	if(texture.getSize().x == 0.f)
		texture.loadFromFile("res/circle.png");
	m_renderBody.setTexture(&texture);
}

void CircleShape::Render(sf::RenderWindow& window)
{
	window.draw(m_renderBody);
}
