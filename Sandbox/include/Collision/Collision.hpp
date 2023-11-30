#pragma once
#include "Dynamics/RigidBody.hpp"

#include <unordered_map>
#include <unordered_set>

class Collision {
public:
	static void Update();

	//static void Render(); //debug only

	//static void AddBody(Collider* body);

	//static void RemoveBody(int index);

	/*static std::unordered_map<int, Collider*> GetBodies() {
		return m_bodies;
	};*/
private:
	static bool CheckCollision(RigidBody* b1, RigidBody* b2, sf::Vector2f& mtv);
private:
	friend class RigidBody;
	//static int m_count;
	//static std::unordered_map<int, Collider*> m_bodies;
	static std::unordered_set<RigidBody*> m_bodiesToUpdate;
};