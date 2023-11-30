#pragma once
#include "Dynamics/RigidBody.hpp"
#include "Collision/DynamicTree.hpp"

#include <unordered_map>
#include <unordered_set>

class Collision {
public:
	static void Update();
	 
	static void Render(sf::RenderWindow& window); //debug only

	static void AddBody(RigidBody* body);

	static void RemoveBody(int index);
private:
	static bool CheckCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv);
	static void ResolveCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv);
private:
	friend class RigidBody;
	static DynamicTree m_Tree;
	static std::unordered_set<RigidBody*> m_bodiesToUpdate;
};