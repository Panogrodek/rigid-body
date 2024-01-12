#pragma once
#include "Dynamics/RigidBody.hpp"

/*
DISCLAIMER:
this part of the code is from FlatPhysics engine (a c# physics engine tutorial from twobitcoder101
the collision resolution i.e. finding contact points and applying forces comes directly from:

Finding contact point -> FlatManifold.cs (https://github.com/twobitcoder101/FlatPhysics-part-23/blob/main/FlatManifold.cs)
Resolving collision with forces -> FlatWorld.cs (https://github.com/twobitcoder101/FlatPhysics-part-23/blob/main/FlatWorld.cs)

MIT License

Copyright (c) 2021 twobitcoder101

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

struct CollisionManifold {
	RigidBody* A;
	RigidBody* B;

	sf::Vector2f mtv = {};

	sf::Vector2f cp1 = {};
	sf::Vector2f cp2 = {};

	uint16_t count = 0;

	void FindContactPoints();
private:
	void FindPolyPolyContactPoint();
	void FindPolyCircleContactPoint(bool polyFirst);
	void FindCircleCircleContactPoint();
};