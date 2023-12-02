#include "stdafx.h"
#include "SatCollision.hpp"
#include "Dynamics/RigidBody.hpp"

#pragma warning(disable : 6386)

SATCollision SATCollision::Instance;

bool SATCollision::SatCollision(RigidBody& body, RigidBody& other, sf::Vector2f& MTV) {
	float minOverlap = INF;

	std::vector<sf::Vector2f> bodyVertices = body.GetTransformedVertices();
	std::vector<sf::Vector2f> otherVertices = other.GetTransformedVertices();

	uint32_t bodyCount = body.m_transformedVertices.size();
	uint32_t otherCount = other.m_transformedVertices.size();

	uint32_t all = bodyCount + otherCount;

	if (all == 2)
		return CircleCollision(body, other, MTV);
	else if (bodyCount == 1)
		return CircleCollision(body, other, MTV);
	else if (otherCount == 1)
		return CircleCollision(body, other, MTV);

	sf::Vector2f* axis = new sf::Vector2f[all];

	for (uint32_t i = 0; i < bodyCount; i++)
		axis[i] = PerpendicularAxis(bodyVertices, i);

	for (uint32_t i = 0; i < otherCount; i++)
		axis[i + bodyCount] = PerpendicularAxis(otherVertices, i);

	for (uint32_t i = 0; i < all; i++) {
		auto& a = axis[i];

		sf::Vector2f bodyProjection = ProjectOnto(bodyVertices, a);
		sf::Vector2f otherProjection = ProjectOnto(otherVertices, a);

		float overlap = Overlap(bodyProjection, otherProjection);

		if (!overlap) {
			MTV = sf::Vector2f(0.0f, 0.0f);

			delete[] axis;

			return false;
		}
		else {
			if (overlap < minOverlap) {
				minOverlap = overlap;

				MTV = a * overlap;
			}
		}
	}

	if (DotProduct(GetCenter(body) - GetCenter(other), MTV) < 0.0f)
		MTV *= -1.0f;

	delete[] axis;
	return true;
}

bool SATCollision::CircleCollision(const RigidBody& body, const RigidBody& other, sf::Vector2f& MTV)
{
	return false;
}

//bool SATCollision::SatCollision(const CircleCollider& body, const RectangleCollider& other, sf::Vector2f& MTV) {
//	//float minOverlap = INF;
//
//	//sf::Vector2f center = body.GetPosition();
//	//float		 radius = body.GetRadius();
//
//	//sf::Vector2f* vert = other.m_vertices;
//
//	//uint32_t vertCount = other.m_verticesCount;
//	//uint32_t all = vertCount + 1;
//
//	//sf::Vector2f* axis = new sf::Vector2f[all];
//
//	//for (uint32_t i = 0; i < vertCount; i++)
//	//	axis[i] = PerpendicularAxis(vert, i, vertCount);
//
//	//axis[vertCount] = CircleAxis(vert, vertCount, center);
//
//	//for (uint32_t i = 0; i < all; i++) {
//	//	auto& a = axis[i];
//
//	//	sf::Vector2f circleProjection = ProjectCircle(center, radius, a);
//	//	sf::Vector2f otherProjection = ProjectOnto(vert, vertCount, a);
//
//	//	float overlap = Overlap(circleProjection, otherProjection);
//
//	//	if (!overlap) {
//	//		MTV = sf::Vector2f(0.0f, 0.0f);
//
//	//		delete[] axis;
//
//	//		return false;
//	//	}
//	//	else {
//	//		if (overlap < minOverlap) {
//	//			minOverlap = overlap;
//
//	//			MTV = a * overlap;
//	//		}
//	//	}
//	//}
//
//	//if (DotProduct(center - GetCenter(other), MTV) < 0.0f)
//	//	MTV *= -1.0f;
//
//	//delete[] axis;
//	return true;
//}

//bool SATCollision::SatCollision(const RectangleCollider& body, const CircleCollider& other, sf::Vector2f& MTV) {
//	bool temp = SatCollision(other, body, MTV);
//	MTV *= -1.0f;
//
//	return temp;
//}

//bool SATCollision::SatCollision(const CircleCollider& body, const CircleCollider& other, sf::Vector2f& MTV) {
//	//sf::Vector2f diff(body.GetPosition() - other.GetPosition());
//
//	//float length = Length(diff);
//	//float sum = body.GetRadius() + other.GetRadius();
//
//	//if (length >= sum)
//	//	return false;
//
//	//sum -= length;
//
//	//diff = Normalize(diff);
//
//	//MTV = diff * sum;
//
//	return true;
//}

sf::Vector2f SATCollision::GetCenter(const RigidBody& body) const {
	sf::Vector2f center;

	uint32_t count = body.m_transformedVertices.size();

	float A = 0.0f;

	float area = 0.0f;

	for (uint32_t i = 0; i < count; i++) {
		auto& v0 = body.m_transformedVertices[i];
		auto& v1 = body.m_transformedVertices[i + 1 != count ? i + 1 : 0];

		float b = CrossProduct(v0, v1);

		A += b;

		center += (v0 + v1) * b;
	}

	center *= 1.0f / (3.0f * A);

	return center;
}

SATCollision::SATCollision()
{
}

SATCollision::~SATCollision()
{
}

sf::Vector2f SATCollision::CircleAxis(sf::Vector2f* vertices, uint32_t count, sf::Vector2f center) {
	sf::Vector2f axis;

	uint32_t index = 0;
	float	 dist = INF;

	for (uint32_t i = 0; i < count; i++) {
		auto& v = vertices[i];

		float d = Distance(v, center);

		if (d >= dist)
			continue;

		dist = d;
		index = i;
	}

	return Normalize(center - vertices[index]);
}

sf::Vector2f SATCollision::PerpendicularAxis(std::vector<sf::Vector2f> vertices, uint32_t index) const {
	uint32_t i0 = index + 1;
	uint32_t i1 = index;
	uint32_t count = vertices.size();


	if (i0 == count) {
		i0 = 0;
		i1 = count - 1;
	}

	sf::Vector2f v = Normalize(vertices[i0] - vertices[i1]);

	return
	{
		-v.y,
		 v.x
	};
}

sf::Vector2f SATCollision::ProjectOnto(std::vector<sf::Vector2f> vertices, sf::Vector2f axis) const {
	uint32_t count = vertices.size();
	float min = INF;
	float max = -INF;

	for (uint32_t i = 0; i < count; i++) {
		float projection = DotProduct(vertices[i], axis);

		if (projection < min)
			min = projection;

		if (projection > max)
			max = projection;
	}

	return
	{
		min,
		max
	};
}

sf::Vector2f SATCollision::ProjectCircle(sf::Vector2f center, float radius, sf::Vector2f axis) const {
	sf::Vector2f dir = radius * axis;

	sf::Vector2f p1 = center + dir;
	sf::Vector2f p2 = center - dir;

	float min = DotProduct(p1, axis);
	float max = DotProduct(p2, axis);

	if (min > max)
		std::swap(min, max);

	return
	{
		min,
		max
	};
}

float SATCollision::Overlap(sf::Vector2f v0, sf::Vector2f v1) const {
	return !(v0.x <= v1.y && v0.y >= v1.x) ? 0.0f : std::min(v0.y, v1.y) - std::max(v0.x, v1.x);
}

float SATCollision::Length(sf::Vector2f v) const
{
	return sqrt(v.x * v.x + v.y * v.y);
}

float SATCollision::Distance(sf::Vector2f v1, sf::Vector2f v2) const
{
	return sqrt(std::pow(v2.x-v1.x,2) + std::pow(v2.y-v1.y,2));
}

float SATCollision::DotProduct(sf::Vector2f v1, sf::Vector2f v2) const
{
	return (v1.x * v2.x + v1.y * v2.y);
}

float SATCollision::CrossProduct(sf::Vector2f v1, sf::Vector2f v2) const
{
	return (v1.x * v2.y - v1.y * v2.x);
}

sf::Vector2f SATCollision::Normalize(sf::Vector2f v) const
{
	return v /= v.length();
}

#pragma warning(default : 6386)