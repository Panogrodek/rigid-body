#include "stdafx.h"
#include "RigidBody.hpp"
#include "Collision/Collision.hpp"

RigidBody::RigidBody(sf::Vector2f pos, RigidBodyData data,
	bool isStatic, float radius, float width, float height, BODY_SHAPE type)
{
	m_position = pos;
	m_linearVelocity = sf::Vector2f{ 0.f,0.f };
	m_rotation = 0.f;
	m_rotationalVelocity = 0.f;
	
	m_data = data;

	m_force = sf::Vector2f{ 0.f,0.f };

	m_isStatic = isStatic;
	m_radius = radius;
	m_width = width;
	m_height = height;
	m_shape = type;

	if (!m_isStatic) {
		CalculateRotationalInertia();
	}
	else {
		m_data.Mass = 0.f;
		m_data.Inertia = 0.f;
	}

	m_data.InvMass = m_data.Mass == 0.f ? 0.f : 1.f / m_data.Mass;
	m_data.InvInertia = m_data.Inertia == 0.f ? 0.f : 1.f / m_data.Inertia;

	if (type == BODY_SHAPE::Box) {
		CreateBoxVertices();
		CreateBoxTriangles();

		m_transformedVertices = m_vertices;
	}
	else if (type == BODY_SHAPE::Circle) {
		CreateCircleBody(m_position, m_radius, m_isStatic);
	}

	m_transformRequired = true;
	m_aabbUpdateRequired = true;

	GetTransformedVertices();
}

void RigidBody::CreateBoxBody(sf::Vector2f pos, float width, float height, bool isStatic) {
	m_shape = BODY_SHAPE::Box;
	m_width = width;
	m_height = height;
	m_position = pos;
	m_isStatic = isStatic;
	CreateBoxVertices();
	CreateBoxTriangles();

	m_transformedVertices = m_vertices;

	m_transformRequired = true;
	m_aabbUpdateRequired = true;

	GetTransformedVertices();
}

void RigidBody::CreateCircleBody(sf::Vector2f pos, float radius, bool isStatic) {
	m_shape = BODY_SHAPE::Circle;
	m_radius = radius;
	m_isStatic = isStatic;
	m_position = pos;

	m_vertices.push_back({0.f,0.f});

	m_transformedVertices = m_vertices;

	m_transformRequired = true;
	m_aabbUpdateRequired = true;

	GetTransformedVertices();
}

void RigidBody::Step(float t)
{
	if (m_isStatic) {
		GetTransformedVertices();
		return;
	}

	m_linearVelocity += m_force/m_data.Mass * t;
	//m_linearVelocity.y -= 9.81f * t;
	m_position += m_linearVelocity * t;

	m_rotation += (m_rotationalVelocity * t) * 180.f/PI;

	Collision::m_bodiesToUpdate.insert(this);

	m_force = sf::Vector2f{ 0.f,0.f };
	m_transformRequired = true;
	m_aabbUpdateRequired = true;

	GetTransformedVertices();
}

void RigidBody::Render(sf::RenderWindow& window)
{
	sf::RectangleShape body;
	static sf::Texture texture;
	switch (m_shape)
	{
	case BODY_SHAPE::Circle:
		body.setSize(sf::Vector2f(2.f * m_radius, 2.f * m_radius));
		body.setOrigin(body.getSize() / 2.f);
		body.setPosition(m_position);
		body.setFillColor(color);
		if (texture.getSize().x == 0.f)
			texture.loadFromFile("res/circle.png");
		body.setTexture(&texture);

		window.draw(body);
		break;
	default:
		std::vector<sf::Vertex> m_vertexes;
		std::vector<sf::Vector2f> m_vertices = GetTransformedVertices();
		for (auto& v : m_triangles)
			m_vertexes.push_back(sf::Vertex(m_vertices[v],color));

		window.draw(m_vertexes.data(), m_vertexes.size(), sf::PrimitiveType::Triangles);
		break;
	}


}

void RigidBody::SetPosition(sf::Vector2f positionVec)
{
	m_position = positionVec;
	Collision::m_bodiesToUpdate.insert(this);
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

void RigidBody::Move(sf::Vector2f moveVec)
{
	m_position += moveVec;
	Collision::m_bodiesToUpdate.insert(this);
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

void RigidBody::Rotate(float angle)
{
	m_rotation += angle;
	Collision::m_bodiesToUpdate.insert(this);
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

void RigidBody::AddImpulse(sf::Vector2f impulse)
{
	m_force = impulse;
}

sf::Vector2f RigidBody::GetPosition()
{
	return m_position;
}

sf::Vector2f RigidBody::GetVelocity()
{
	return m_linearVelocity;
}

void RigidBody::CreateBoxVertices()
{
	float left = -m_width / 2.f;
	float right = left + m_width;
	float bottom = -m_height / 2.f;
	float top = bottom + m_height;

	std::vector<sf::Vector2f> vertices;
	vertices.push_back(sf::Vector2f(left,	top));
	vertices.push_back(sf::Vector2f(right,	top));
	vertices.push_back(sf::Vector2f(right,	bottom));
	vertices.push_back(sf::Vector2f(left,	bottom));

	m_vertices = vertices;
}

void RigidBody::CreateBoxTriangles()
{
	std::vector<int> triangles;
	triangles.push_back(0);
	triangles.push_back(1);
	triangles.push_back(2);
	triangles.push_back(0);
	triangles.push_back(2);
	triangles.push_back(3);
	m_triangles = triangles;
}

BODY_SHAPE RigidBody::GetShape()
{
	return m_shape;
}

AABB RigidBody::GetAABB()
{
	if (m_aabbUpdateRequired)
	{
		sf::Vector2f min;
		sf::Vector2f max;
		if (m_shape == BODY_SHAPE::Circle)
		{
			min = { m_position.x - m_radius, m_position.y - m_radius};
			max = { m_position.x + m_radius, m_position.y + m_radius};
		}
		else
		{
			float minX =  INFINITY;
			float minY =  INFINITY;
			float maxX = -INFINITY;
			float maxY = -INFINITY;
			for (auto& v : m_transformedVertices)
			{
				if (v.x < minX) { minX = v.x; }
				if (v.x > maxX) { maxX = v.x; }
				if (v.y < minY) { minY = v.y; }
				if (v.y > maxY) { maxY = v.y; }
			}
			min = { minX, minY };
			max = { maxX, maxY };
		}

		m_aabb.lowerBound = min;
		m_aabb.upperBound = max;
	}

	m_aabbUpdateRequired = false;
	return m_aabb;
}

std::vector<sf::Vector2f> RigidBody::GetTransformedVertices()
{
	if (m_transformRequired) //this is so fucking stupid im gonna cry
	{	
		float sin = sinf(m_rotation * PI/180.f );
		float cos = cosf(m_rotation * PI/180.f );

		for (int i = 0; i < m_vertices.size(); i++)
		{
			auto& v = m_vertices[i];
			float rx = cos * v.x - sin * v.y;
			float ry = sin * v.x + cos * v.y;

			m_transformedVertices[i] = sf::Vector2f(rx + m_position.x, ry + m_position.y);
		}
	}

	m_transformRequired = false;
	return m_transformedVertices;
}

bool AABB::contains(sf::Vector2f point)
{
	if (point.x < lowerBound.x || point.x > upperBound.x)
		return false;
	if (point.y < lowerBound.y || point.y > upperBound.y)
		return false;
	return true;
}

bool AABB::contains(AABB other)
{
	return contains(other.lowerBound) && contains(other.upperBound);
}

bool AABB::intersects(AABB other)
{
	return (lowerBound.x <= other.upperBound.x && upperBound.x >= other.lowerBound.x) &&
		(lowerBound.y <= other.upperBound.y && upperBound.y >= other.lowerBound.y);
}

float AABB::GetPerimeter()
{
	sf::Vector2f d = upperBound - lowerBound;
	return d.x * d.y;
}

float AABB::GetArea()
{
	sf::Vector2f d = upperBound - lowerBound;

	return 2.0f * d.x * d.y;
}

void RigidBody::CalculateRotationalInertia()
{
	switch (m_shape)
	{
	case BODY_SHAPE::Circle:
		m_data.Inertia = (1.f / 2.f) * m_data.Mass * m_radius * m_radius;
		break;
	case BODY_SHAPE::Box:
		m_data.Inertia = (1.f / 12.f) * m_data.Mass * (m_width * m_width + m_height * m_height);
		break;
	default:
		break;
	}
}
