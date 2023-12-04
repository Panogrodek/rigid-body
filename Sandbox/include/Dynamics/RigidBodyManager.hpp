#pragma once
#include "RigidBody.hpp"
#include <unordered_map>

class RigidBodyManager {
public:
	static void Init();

	static void AddBody(RigidBody* body);

	static void DeleteBody(int index);
	static void DeleteBody(RigidBody* body);

	static void Destroy();

	static void Update(float t);
	static void Render(sf::RenderWindow& window);
	static int GetCount();

	static std::unordered_map<int, RigidBody*>& GetBodies();
protected:
	friend class RigidBody;
	friend class Collision;

private:
	static int m_count;
	static std::unordered_map<int,RigidBody*> m_bodies;
	static std::vector<sf::RectangleShape> m_points;
};