#include "stdafx.h"
#include "CollisionManifold.hpp"

#include "Math.hpp"

void CollisionManifold::FindContactPoints()
{
	if (A->m_shape == BODY_SHAPE::Box) {
		if (B->m_shape == BODY_SHAPE::Box)
			FindPolyPolyContactPoint();
		else if (B->m_shape == BODY_SHAPE::Circle)
			FindPolyCircleContactPoint(true);
	}
	else if (A->m_shape == BODY_SHAPE::Circle) {
		if (B->m_shape == BODY_SHAPE::Box)
			FindPolyCircleContactPoint(false);
		else if (B->m_shape == BODY_SHAPE::Circle)
			FindCircleCircleContactPoint();
	}
}

void CollisionManifold::FindPolyCircleContactPoint(bool polyFirst)
{
	std::vector<sf::Vector2f> polygonVertices = polyFirst ? A->GetTransformedVertices() : B->GetTransformedVertices();
	sf::Vector2f circleVertex = polyFirst ? B->GetTransformedVertices()[0] : A->GetTransformedVertices()[0];
	float minDistSq = INFINITY;

	for (int i = 0; i < polygonVertices.size(); i++)
	{
		sf::Vector2f la = polygonVertices[i];
		sf::Vector2f lb = polygonVertices[(i + 1) % polygonVertices.size()];

		sf::Vector2f contact{};
		float distSq = PointLineDistance(la, lb, circleVertex, contact);

		if (distSq < minDistSq)
		{
			minDistSq = distSq;
			cp1 = contact;
			count = 1;
		}
	}
}

void CollisionManifold::FindCircleCircleContactPoint()
{
	sf::Vector2f ab = A->GetTransformedVertices()[0] - B->GetTransformedVertices()[0];
	sf::Vector2f dir = Normalize(ab);
	cp1 = A->GetTransformedVertices()[0] + dir * A->m_radius;
	count = 1;
}

void CollisionManifold::FindPolyPolyContactPoint()
{
	auto& va = A->GetTransformedVertices();
	auto& vb = B->GetTransformedVertices();
	float minDistSq = INFINITY;

	for (auto& p : va)
	{
		for (int j = 0; j < vb.size(); j++)
		{
			sf::Vector2f la = vb[j];
			sf::Vector2f lb = vb[(j + 1) % vb.size()];

			sf::Vector2f cp{};

			float distSq = PointLineDistance(la, lb, p, cp);

			if (NearlyEqual(distSq, minDistSq))
			{
				if (!NearlyEqual(cp, cp1))
				{
					cp2 = cp;
					count = 2;
				}
			}
			else if (distSq < minDistSq)
			{
				minDistSq = distSq;
				count = 1;
				cp1 = cp;
			}
		}
	}

	for (auto& p : vb)
	{
		for (int j = 0; j < va.size(); j++)
		{
			sf::Vector2f la = va[j];
			sf::Vector2f lb = va[(j + 1) % va.size()];

			sf::Vector2f cp{};

			float distSq = PointLineDistance(la, lb, p, cp);

			if (NearlyEqual(distSq, minDistSq))
			{
				if (!NearlyEqual(cp, cp1))
				{
					cp2 = cp;
					count = 2;
				}
			}
			else if (distSq < minDistSq)
			{
				minDistSq = distSq;
				count = 1;
				cp1 = cp;
			}
		}
	}
}
