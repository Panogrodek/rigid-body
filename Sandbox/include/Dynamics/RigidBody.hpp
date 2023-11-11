#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/RenderWindow.hpp>

constexpr int INVALID_ID = -1;

enum class BODY_TYPE	{ None = 0, Static = 1, Kinematic = 2, Dynamic = 3 };
enum class BODY_SHAPE	{ None = 0, Circle = 1, Rectangle = 2};

struct AABB {
	sf::Vector2f lowerBound;
	sf::Vector2f upperBound;

	bool contains(sf::Vector2f point);
	bool contains(AABB other);
	bool intersects(AABB other);
	float GetPerimeter();
	float GetArea();
};

class RigidBody {
public:
	void Update(float t);

	void Render(sf::RenderWindow& window);
	
	//Modifiers
	void Move(sf::Vector2f moveVec);
	void Scale(float scalar);
	void Rotate(float angle);

	void SetPosition	(sf::Vector2f positionVec);
	void SetVelocity	(sf::Vector2f velocityVec);
	void SetAcceleration(sf::Vector2f accelerationVec);

	//Accessors
	BODY_TYPE GetType();
	BODY_SHAPE GetShape();

	AABB GetAABB();

	sf::Vector2f GetPosition();
	sf::Vector2f GetVelocity();
	sf::Vector2f GetAcceleration();
protected:
	sf::Vector2f m_position;
	sf::Vector2f m_velocity;
	sf::Vector2f m_acceleration;
	float m_mass;

	AABB m_aabb;


	sf::RectangleShape m_renderBody;
	BODY_TYPE m_type	= BODY_TYPE::None;
	BODY_SHAPE m_shape  = BODY_SHAPE::None;
private:
	friend class RigidBodyManager;
	int m_id = INVALID_ID;
};