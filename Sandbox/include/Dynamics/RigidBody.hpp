#pragma once
#include "Collision/AABB.hpp"
#include "SFML/System/Vector2.hpp"
#include "SFML/Graphics/RectangleShape.hpp"

#include <functional>
constexpr int INVALID_ID = -1;

enum class BODY_SHAPE { None = -1, Circle = 0, Box = 1, };

struct RigidBodyPhysicsData {
	float Mass = 0.f;
	float Restitution = 0.f;
	float Inertia = 0.f;
private:
	friend class RigidBody;
	friend class Collision;
	float InvMass = 0.f;
	float InvInertia = 0.f;
};

class RigidBody {
public:
	RigidBody() {};
	RigidBody(sf::Vector2f pos, RigidBodyPhysicsData data,
		float radius, float width, float height, bool isStatic = false, BODY_SHAPE type = BODY_SHAPE::None);

	//Update
	void Step(float t);

	//Drawing
	void Render(sf::RenderWindow& window);

	//Modifiers
	void Move(const sf::Vector2f& moveVec);
	void Rotate(const sf::Angle& angle);
	void SetPosition(const sf::Vector2f& positionVec);
	void SetRotation(const sf::Angle& angle);

	//linear movement
	void AddLinearImpulse(const sf::Vector2f& impulse);
	void AddLinearForce(const sf::Vector2f& force);
	void SetVelocity(const sf::Vector2f& velocity);
	void ClearLinearForce();

	//angular movement
	void AddRotationalImpulse(const sf::Angle& impulse);
	void AddRotationalForce(const sf::Angle& rotationalForce);
	void SetRotationalVelocity(const sf::Angle& velocity);
	void ClearRotationalForce();

	void ClearAllForces();

	//Accessors
	BODY_SHAPE GetShape();
	sf::Vector2f GetPosition();
	sf::Vector2f GetVelocity();
	sf::Vector2f GetSize();
	sf::Angle GetRotation();
	sf::Angle GetRotationalVelocity();

	//User Data
	void SetOnIntersection(std::function<void(RigidBody*)> fun);

	//Creators
	void CreateBoxBody(sf::Vector2f pos, float width, float height, RigidBodyPhysicsData data, bool isStatic);
	void CreateCircleBody(sf::Vector2f pos, float radius, RigidBodyPhysicsData data, bool isStatic);

	//Rendering
	sf::RectangleShape& GetRenderBody();
private:
	AABB GetAABB();

	//physics
	void CalculateMassAndInertia();
	void CalculateRotationalInertia();

	//vertices
	void UpdateVertices();
	void ClearVertexData();
	std::vector<sf::Vector2f> GetTransformedVertices();

	//box body
	void CreateBoxVertices();
private:
	//vertex data
	sf::Vector2f m_position{};
	sf::Angle m_rotation{};
	std::vector<sf::Vector2f> m_vertices;
	std::vector<sf::Vector2f> m_transformedVertices;

	//user data
	std::function<void(RigidBody*)> m_onIntersection = 0;
	//velocities
	float m_rotationalVelocity = 0.f;
	sf::Vector2f m_linearVelocity{ 0.f,0.f };

	//forces
	sf::Angle m_rotationalForce{};
	sf::Vector2f m_linearForce{ 0.f,0.f };

	//impulses
	sf::Angle m_rotationalImpulse{};
	sf::Vector2f m_linearImpulse{ 0.f,0.f };

	AABB m_aabb{};
	RigidBodyPhysicsData m_data{};

	//dimensions
	float m_radius = 0.f;
	float m_height = 0.f;
	float m_width = 0.f;

	bool m_isStatic = false;
	BODY_SHAPE m_shape = BODY_SHAPE::None;

	bool m_transformRequired = true;
	bool m_aabbUpdateRequired = true;
	//memory
	int m_nodeIndex = INVALID_ID;
	int m_id = INVALID_ID;

	//rendering
	sf::RectangleShape m_renderBody;

	//friends
	friend class Collision;
	friend class CollisionManifold;
	friend class SATCollision;
	friend class RigidBodyManager;
	friend class DynamicTree;
};