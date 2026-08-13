/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <map>
#include "MathUtils.h"
#include "Arbiter.h"

struct Body;
struct Joint;

struct Impulse
{
    Vec2 position;
    Vec2 velocity;
};

struct World
{
public:
	static bool accumulateImpulses;
	static bool warmStarting;
	static bool positionCorrection;

	Vec2 gravity;
	int iterations;

    std::vector<Body*> bodies;
	std::vector<Joint*> joints;
	std::map<ArbiterKey, Arbiter> arbiters;
    std::vector<Impulse> impulse_s;

    int selectedBodyIndex;
    Vec2 selectedBodyOffset;
    Vec2 selectedBodyPoint;

	World(Vec2 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

	void Clear();
	void Add(Body* body);
	void Add(Joint* joint);
    void Add(Impulse impulse);
    void SelectBody(Vec2 mousePos);

	void Step(float dt);

private:
	void BroadPhase();
};

#endif
