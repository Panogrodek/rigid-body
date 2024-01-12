#include "stdafx.h"
#include "RigidBody.hpp"
#include "Math.hpp"
#include "Collision/Collision.hpp"

RigidBody::RigidBody(sf::Vector2f pos, RigidBodyPhysicsData data,
	float radius, float width, float height, bool isStatic, BODY_SHAPE shape) :
	m_position(pos),
	m_data(data),
	m_radius(radius),
	m_width(width),
	m_height(height),
	m_isStatic(isStatic),
	m_shape(shape)
{
	CalculateMassAndInertia();

	if (m_shape == BODY_SHAPE::Box)
		CreateBoxVertices();
	else if (m_shape == BODY_SHAPE::Circle)
		m_vertices.push_back({ 0.f,0.f });

	if (m_shape == BODY_SHAPE::Circle) {
		static sf::Texture texture;
		if (texture.getSize().x == 0.f)
			texture.loadFromFile("res/circle.png");
		m_renderBody.setTexture(&texture);
	}

	m_transformedVertices = m_vertices;

	UpdateVertices();
}

void RigidBody::CreateBoxBody(sf::Vector2f pos, float width, float height, RigidBodyPhysicsData data, bool isStatic) {
	ClearVertexData();
	ClearAllForces();

	m_shape = BODY_SHAPE::Box;
	m_position = pos;
	m_width = width;
	m_height = height;
	m_data = data;
	m_isStatic = isStatic;

	CalculateMassAndInertia();

	CreateBoxVertices();
	m_transformedVertices = m_vertices;

	m_transformRequired = true;
	m_aabbUpdateRequired = true;

	GetTransformedVertices();
}

void RigidBody::CreateCircleBody(sf::Vector2f pos, float radius, RigidBodyPhysicsData data, bool isStatic) {
	ClearVertexData();
	ClearAllForces();

	m_shape = BODY_SHAPE::Circle;
	m_position = pos;
	m_radius = radius;
	m_data = data;
	m_isStatic = isStatic;

	static sf::Texture texture;
	if (texture.getSize().x == 0.f)
		texture.loadFromFile("res/textures/circle.png");
	m_renderBody.setTexture(&texture);

	CalculateMassAndInertia();

	m_vertices.push_back({ 0.f,0.f });
	m_transformedVertices = m_vertices;

	m_transformRequired = true;
	m_aabbUpdateRequired = true;

	GetTransformedVertices();
}

void RigidBody::Step(float t)
{
	if (m_isStatic) {
		UpdateVertices();
		return;
	}

	m_linearVelocity += m_linearImpulse * t;
	m_linearVelocity += m_linearForce * t;

	m_rotationalVelocity += m_rotationalImpulse.asRadians() * m_data.InvInertia * t;
	m_rotationalVelocity += m_rotationalForce.asRadians() * t;

	m_rotation += sf::radians(m_rotationalVelocity * t);

	m_position += m_linearVelocity * t;
	Collision::m_bodiesToUpdate.insert(this);

	m_rotationalImpulse = sf::degrees(0.f);
	m_linearImpulse = { 0.f,0.f };

	m_transformRequired = true;
	m_aabbUpdateRequired = true;
	UpdateVertices();
}

void RigidBody::Render(sf::RenderWindow& window)
{
	window.draw(m_renderBody);
}

//Modifiers
void RigidBody::Move(const sf::Vector2f& moveVec)
{
	m_position += moveVec;
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

void RigidBody::Rotate(const sf::Angle& angle)
{
	m_rotation += angle;
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

void RigidBody::SetPosition(const sf::Vector2f& positionVec)
{
	m_position = positionVec;
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

void RigidBody::SetRotation(const sf::Angle& angle)
{
	m_rotation = angle;
	m_aabbUpdateRequired = true;
	m_transformRequired = true;
}

//Physics
void RigidBody::AddLinearImpulse(const sf::Vector2f& impulse)
{
	m_linearImpulse += impulse;
}

void RigidBody::AddLinearForce(const sf::Vector2f& force)
{
	m_linearForce += force;
}

void RigidBody::SetVelocity(const sf::Vector2f& velocity) {
	m_linearVelocity = velocity;
}

void RigidBody::ClearLinearForce()
{
	m_linearForce = { 0.f,0.f };
}

void RigidBody::AddRotationalImpulse(const sf::Angle& impulse)
{
	m_rotationalImpulse += impulse;
}

void RigidBody::AddRotationalForce(const sf::Angle& rotationalForce)
{
	m_rotationalForce += rotationalForce;
}

void RigidBody::SetRotationalVelocity(const sf::Angle& velocity) {
	m_rotationalVelocity = velocity.asRadians();
}

void RigidBody::ClearRotationalForce()
{
	m_rotationalForce = sf::degrees(0.f);
}

void RigidBody::ClearAllForces()
{
	m_linearForce = { 0.f,0.f };
	m_rotationalForce = sf::degrees(0.f);

	m_linearImpulse = { 0.f,0.f };
	m_rotationalImpulse = sf::degrees(0.f);
}

//Accessors
sf::Vector2f RigidBody::GetPosition()
{
	return m_position;
}

sf::Vector2f RigidBody::GetVelocity()
{
	return m_linearVelocity;
}

sf::Vector2f RigidBody::GetSize()
{
	if (m_shape == BODY_SHAPE::Box)
		return sf::Vector2f(m_width, m_height);
	else
		return sf::Vector2f(2.f * m_radius, 2.f * m_radius);
}

sf::Angle RigidBody::GetRotation()
{
	return sf::Angle(m_rotation);
}

sf::Angle RigidBody::GetRotationalVelocity() {
	return sf::radians(m_rotationalVelocity);
}

void RigidBody::SetOnIntersection(std::function<void(RigidBody*)> fun)
{
	m_onIntersection = fun;
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
			min = { m_position.x - m_radius, m_position.y - m_radius };
			max = { m_position.x + m_radius, m_position.y + m_radius };
		}
		else
		{
			float minX = INFINITY;
			float minY = INFINITY;
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

sf::RectangleShape& RigidBody::GetRenderBody() {
	return m_renderBody;
}

std::vector<sf::Vector2f> RigidBody::GetTransformedVertices()
{
	UpdateVertices();
	return m_transformedVertices;
}

void RigidBody::CreateBoxVertices()
{
	float left = -m_width / 2.f;
	float right = left + m_width;
	float bottom = -m_height / 2.f;
	float top = bottom + m_height;

	std::vector<sf::Vector2f> vertices;
	vertices.push_back(sf::Vector2f(left, top));
	vertices.push_back(sf::Vector2f(right, top));
	vertices.push_back(sf::Vector2f(right, bottom));
	vertices.push_back(sf::Vector2f(left, bottom));

	m_vertices = vertices;
}

void RigidBody::CalculateMassAndInertia()
{
	//Calculate Mass and inertia
	if (!m_isStatic) {
		CalculateRotationalInertia();
	}
	else {
		m_data.Mass = 0.f;
		m_data.Inertia = 0.f;
	}

	m_data.InvMass = m_data.Mass == 0.f ? 0.f : 1.f / m_data.Mass;
	m_data.InvInertia = m_data.Inertia == 0.f ? 0.f : 1.f / m_data.Inertia;
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

void RigidBody::UpdateVertices()
{
	if (m_transformRequired)
	{
		float sin = sinf(m_rotation.asRadians());
		float cos = cosf(m_rotation.asRadians());

		for (int i = 0; i < m_vertices.size(); i++)
		{
			auto& v = m_vertices[i];
			float rx = cos * v.x - sin * v.y;
			float ry = sin * v.x + cos * v.y;

			m_transformedVertices[i] = sf::Vector2f(rx + m_position.x, ry + m_position.y);
		}

		if (m_nodeIndex != INVALID_ID)
			Collision::m_bodiesToUpdate.insert(this);

		//Only for rendering
		switch (m_shape)
		{
		case BODY_SHAPE::Circle:
			m_renderBody.setSize(GetSize());
			m_renderBody.setOrigin(GetSize() / 2.f);
			m_renderBody.setPosition(GetPosition());
			m_renderBody.setRotation(GetRotation());
			break;
		case BODY_SHAPE::Box:
			m_renderBody.setSize(GetSize());
			m_renderBody.setOrigin(GetSize() / 2.f);
			m_renderBody.setPosition(GetPosition());
			m_renderBody.setRotation(GetRotation());
			break;
		}
	}
	m_transformRequired = false;
}

void RigidBody::ClearVertexData()
{
	m_vertices.clear();
	m_transformedVertices.clear();
	m_position = { 0.f,0.f };
	m_rotation = sf::degrees(0.f);
}
