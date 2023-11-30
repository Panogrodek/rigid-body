#pragma once
#include "Dynamics/RigidBody.hpp"


constexpr float INF = std::numeric_limits<float>::infinity();

class SATCollision {
public:
	static SATCollision Instance;

	bool SatCollision(const RigidBody& body, const RigidBody& other, sf::Vector2f& MTV);
	bool CircleCollision(const RigidBody& body, const RigidBody& other, sf::Vector2f& MTV);
private:
	SATCollision();
	~SATCollision();

	sf::Vector2f GetCenter(const RigidBody& body) const;

	sf::Vector2f CircleAxis(sf::Vector2f* vertices, uint32_t count, sf::Vector2f center);

	sf::Vector2f PerpendicularAxis(std::vector<sf::Vector2f> vertices, uint32_t index) const;

	sf::Vector2f ProjectOnto(std::vector<sf::Vector2f> vertices, sf::Vector2f axis) const;
	sf::Vector2f ProjectCircle(sf::Vector2f circleCenter, float radius, sf::Vector2f axis) const;

	float Overlap(sf::Vector2f v0, sf::Vector2f v1) const;

	//very much temporary
	float Length(sf::Vector2f v) const;
	float Distance(sf::Vector2f v1, sf::Vector2f v2) const;
	float DotProduct(sf::Vector2f v1, sf::Vector2f v2) const;
	float CrossProduct(sf::Vector2f v1, sf::Vector2f v2) const;

	sf::Vector2f Normalize(sf::Vector2f v) const;
};