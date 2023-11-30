#pragma once
#include "Dynamics/RigidBody.hpp"
constexpr float FAT_FACTOR = 2.0f; //determines the fat aabbs size increase

constexpr int MaxObjects = 100000; //TODO: dynamic refitting of the tree

struct Node
{
	AABB box;
	int ownIndex;
	int parentIndex = -1;
	int child1 = -1;
	int child2 = -1;
	bool isLeaf;

	RigidBody* object;
};

class DynamicTree {
public:
	DynamicTree();
	~DynamicTree();
	void Insert(RigidBody* obj);

	void Update(int index);
	void Render(sf::RenderWindow& window);
	void RemoveLeafNode(int index);

	std::vector<RigidBody*> GetCollisions(RigidBody* object);
private:
	friend class Collision;
	Node* m_nodes;
	int m_nodeCount = 0;
	int m_rootIndex = -1;

	void SwapNodeWithLast(int index);
	void RefitTree(int index);
	int AllocateNewLeaf(RigidBody* rect);
	int AllocateInternalNode(Node& a, Node& b);

	AABB Union(AABB a, AABB b);

};