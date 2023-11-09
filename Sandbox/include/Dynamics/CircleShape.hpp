#pragma once

class CircleShape {
public:
	CircleShape() {};
	CircleShape(sf::Vector2f position, float radius);

	void Render(sf::RenderWindow& window);
private:
	sf::RectangleShape m_renderBody;

	sf::Vector2f m_position{};
	float m_radius = 0.f;
};