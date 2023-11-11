#include "stdafx.h"
#include "RigidBody.hpp"

void RigidBody::Move(sf::Vector2f moveVec)
{
	m_position += moveVec;
	m_renderBody.setPosition(m_position);
}

void RigidBody::Scale(float scalar)
{
}

void RigidBody::Rotate(float angle)
{
}

void RigidBody::Update(float t)
{
	//semi-implicid euler
	m_velocity += m_acceleration * t;
	m_position += m_velocity * t;

	m_renderBody.setPosition(m_position);
}

void RigidBody::Render(sf::RenderWindow& window)
{
	window.draw(m_renderBody);
}

void RigidBody::SetPosition(sf::Vector2f positionVec)
{
	m_position = positionVec;
}

void RigidBody::SetVelocity(sf::Vector2f velocityVec)
{
	m_velocity = velocityVec;
}

void RigidBody::SetAcceleration(sf::Vector2f accelerationVec)
{
	m_acceleration = accelerationVec;
}

sf::Vector2f RigidBody::GetPosition()
{
	return m_position;
}

sf::Vector2f RigidBody::GetVelocity()
{
	return m_velocity;
}

sf::Vector2f RigidBody::GetAcceleration()
{
	return m_acceleration;
}

BODY_TYPE RigidBody::GetType()
{
	return m_type;
}

BODY_SHAPE RigidBody::GetShape()
{
	return m_shape;
}

AABB RigidBody::GetAABB()
{
	return m_aabb;
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

