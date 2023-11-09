#include "stdafx.h"
#include "GJK.hpp"
#include "Math.hpp"

size_t indexOfFurthestPoint(const sf::Vector2f* vertices, size_t count, sf::Vector2f direction) {
	size_t index = 0;
	float maxProduct = dotProduct(direction, vertices[index]);
	for (size_t i = 1; i < count; i++) {
		float product = dotProduct(direction, vertices[i]); // may be negative
		if (product > maxProduct) {
			maxProduct = product;
			index = i;
		}
	}
	return index;
}

sf::Vector2f GJK::Support(sf::Vector2f* ver1, uint16_t count1, sf::Vector2f* ver2, uint16_t count2, sf::Vector2f direction)
{
	int i = indexOfFurthestPoint(v1, c1, direction);
	int j = indexOfFurthestPoint(v2, c2, -direction);

	return sf::Vector2f(v1[i] - v2[j]);
}


bool GJK::Check(sf::Vector2f* ver1, uint16_t count1, sf::Vector2f* ver2, uint16_t count2)
{
	v1 = ver1;
	v2 = ver2;
	c1 = count1;
	c2 = count2;
	//rendering
	sf::Color col(255, 255, 0, 255);
	AddVertexes(ver1, count1,col);
	AddVertexes(ver2, count2,col);
	CreateMinowskiSpace();
	//

	//GJK \/
	uint16_t index = 0;

	sf::Vector2f pos1 = averagePoint(v1, c1);
	sf::Vector2f pos2 = averagePoint(v2, c2);
	sf::Vector2f a, b, c = { 0,0 };
	sf::Vector2f ao , ab , ac , acperp, abperp = { 0,0 };

	sf::Vector2f d = { pos1 - pos2 };

	if ((d.x == 0) && (d.y == 0))
		d.x = 1.f;

	a = simplex[0] = Support(v1,c1,v2,c2,d);

	if (dotProduct(a, d) <= 0)
		return 0; // no collision

	d = -a;

	while (true) {
		iter_count++;

		a = simplex[++index] = Support(v1, c1, v2, c2, d);

		if (dotProduct(a, d) <= 0) {
			std::cout << iter_count << "\n";
			return 0; // no collision
		}

		sf::Vector2f ao = negate(a); // from point A to Origin is just negative A
		// simplex has 2 points (a line segment, not a triangle yet)
		if (index < 2) {
			b = simplex[0];
			ab = subtract(b, a); // from point A to B
			d = tripleProduct(ab, ao, ab); // normal to AB towards Origin
			if (lengthSquared(d) == 0)
				d = perpendicular(ab);
			continue; // skip to next iteration
		}

		b = simplex[1];
		c = simplex[0];
		ab = subtract(b, a); // from point A to B
		ac = subtract(c, a); // from point A to C

		acperp = tripleProduct(ab, ac, ac);

		if (dotProduct(acperp, ao) >= 0) {

			d = acperp; // new direction is normal to AC towards Origin

		}
		else {

			abperp = tripleProduct(ac, ab, ab);

			if (dotProduct(abperp, ao) < 0) {
				std::cout << iter_count << "\n";
				return 1; // collision
			}

			simplex[0] = simplex[1]; // swap first element (point C)

			d = abperp; // new direction is normal to AB towards Origin
		}

		simplex[1] = simplex[2]; // swap element in the middle (point B)
		--index;
	}

}

void GJK::Draw(sf::RenderWindow* window, bool ShowMinkSpace)
{
	window->draw(m_Vertexes.data(), m_Vertexes.size(), sf::PrimitiveType::Lines);
	if (ShowMinkSpace) {
		for (auto& s : m_MinkSpaceVer) {
			window->draw(s);
		}
	}

	for (int i = 0; i < 3; i++) {
		sf::RectangleShape s;
		s.setSize({ 0.25f, 0.25f });
		s.setOrigin(s.getSize() / 2.f);
		s.setPosition(simplex[i]);
		window->draw(s);
	}
}

void GJK::AddVertexes(sf::Vector2f* ver, uint16_t count, sf::Color col)
{

	for (uint16_t i = 1; i < count; i++) {
		m_Vertexes.push_back(sf::Vertex({ ver[i - 1].x  ,ver[i - 1].y },col));
		m_Vertexes.push_back(sf::Vertex({ ver[i].x		,ver[i].y	  },col));
	}
	m_Vertexes.push_back(sf::Vertex({ ver[0].x,		   ver[0].y },col));
	m_Vertexes.push_back(sf::Vertex({ ver[count - 1].x,ver[count - 1].y },col));
}

void GJK::CreateMinowskiSpace()
{
	std::vector<sf::Vector2f> m_MinkSpacePoints;
	for (uint16_t j = 0; j < c1; j++) {
		for (uint16_t k = 0; k < c2; k++) {
			m_MinkSpacePoints.push_back({ v1[j] - v2[k] });
		}
	}

	for (auto& ver : m_MinkSpacePoints) {
		sf::RectangleShape s({ 0.125f,0.125f });
		s.setOrigin(s.getSize() / 2.f);

		s.setPosition({ ver.x,ver.y });
		//s.setFillColor(sf::Color(128, 0, 128, 255));
		m_MinkSpaceVer.push_back(s);
	}


}

