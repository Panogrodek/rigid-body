#pragma once
#include "Dynamics/RigidBody.hpp"
#include "Collision/DynamicTree.hpp"

#include <unordered_map>
#include <unordered_set>

struct CollisionManifold {
	RigidBody* A;
	RigidBody* B;

	sf::Vector2f mtv = {};

	sf::Vector2f cp1 = {};
	sf::Vector2f cp2 = {};

	uint16_t count = 0;

	void FindContactPoints();
private:
	void FindContactPoint(std::vector<sf::Vector2f> va, std::vector<sf::Vector2f> vb);
};

class Collision {
public:
	static void Update();
	 
	static void Render(sf::RenderWindow& window); //debug only

	static void AddBody(RigidBody* body);

	static void RemoveBody(int index);
private:
	static bool CheckCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv);
	static void ResolveCollision(CollisionManifold& manifold);
	static void ResolveCollisionWithRotation(CollisionManifold& c);
	static bool DoesCollisionExist(RigidBody* b1, RigidBody* b2);
private:
	friend class RigidBody;
	friend class CollisionManifold;
	static DynamicTree m_Tree;

	static std::unordered_set<RigidBody*> m_bodiesToUpdate;
	static std::vector<CollisionManifold> m_collidingBodies;
	static std::vector<sf::RectangleShape> m_points;
};