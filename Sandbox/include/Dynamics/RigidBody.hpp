#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/RenderWindow.hpp>

constexpr int INVALID_ID = -1;
constexpr float PI = 3.1415926535897932384626433832795f;

enum class BODY_SHAPE	{ Circle = 0, Box = 1,};

struct AABB {
	sf::Vector2f lowerBound;
	sf::Vector2f upperBound;

	bool contains(sf::Vector2f point);
	bool contains(AABB other);
	bool intersects(AABB other);
	float GetPerimeter();
	float GetArea();
};

struct RigidBodyData {
	float Density;
	float Mass;
	float Restitution;
	float InvMass;
protected:
	friend class RigidBody;
	float Area;
};

class RigidBody {
public:
	RigidBody(sf::Vector2f pos, RigidBodyData data, 
		bool isStatic, float radius, float width, float height, BODY_SHAPE type);

	void Step(float t);
	void Render(sf::RenderWindow& window);
	
	//Modifiers
	void Move(sf::Vector2f moveVec);
	void Rotate(float angle);

	void SetPosition	(sf::Vector2f positionVec);
	void AddImpulse		(sf::Vector2f impulse);
	//Accessors
	BODY_SHAPE GetShape();

	sf::Vector2f GetPosition();
	sf::Vector2f GetVelocity();

protected:
	std::vector<sf::Vector2f> GetTransformedVertices();
	AABB GetAABB();
protected:
	sf::Vector2f m_position;
	sf::Vector2f m_linearVelocity;
	float m_rotation;
	float m_rotationalVelocity;

	sf::Vector2f m_force;

	RigidBodyData m_data;

	bool m_isStatic;

	float m_radius;
	float m_height;
	float m_width;

	std::vector<int> m_triangles;

	BODY_SHAPE m_shape;
private:
	AABB m_aabb;
	std::vector<sf::Vector2f> m_transformedVertices;
	std::vector<sf::Vector2f> m_vertices;

	bool m_transformRequired;
	bool m_aabbUpdateRequired;

	void CreateBoxVertices();
	void CreateBoxTriangles();
private:
	friend class Collision;
	friend class SATCollision;
	friend class RigidBodyManager;
	friend class DynamicTree;

	int m_nodeIndex = INVALID_ID;
	int m_id = INVALID_ID;
};