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

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

using std::vector;
using std::map;
using std::pair;

typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

void World::Add(Body* body)
{
	bodies.push_back(body);
}

void World::Add(Joint* joint)
{
	joints.push_back(joint);
}

void World::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}

void World::BroadPhase()
{
	// O(n^2) broad-phase
	for (int i =   0; i < (int)bodies.size(); i++)
    for (int j = i+1; j < (int)bodies.size(); j++)
	{
		Body* bi = bodies[i];
        Body* bj = bodies[j];

        if (bi->invMass == 0.0f && bj->invMass == 0.0f) continue;

        Arbiter newArb(bi, bj);
        ArbiterKey key(bi, bj);

        if (newArb.numContacts == 0)
        {
            arbiters.erase(key);
            continue;
        }

        auto iter = arbiters.find(key);
        bool found = iter != arbiters.end();
        if (!found)
            arbiters.insert(ArbPair(key, newArb));
        else
            iter->second.Update(newArb.contacts, newArb.numContacts);
	}
}

void World::Step(float dt)
{
    // float dti = dt > 0.0f ? 1.0f / dt : 0.0f;

    assert(dt >= 0.0f);
	float dti = 1.0f / dt;

	// determine overlapping bodies and update contact points
	BroadPhase();

	// integrate forces
    for (auto& b : bodies)
	{
		if (b->invMass == 0.0f) continue;

        b->velocity += dt * gravity;
        b->velocity += dt * b->invMass * b->force;
		b->angularVelocity += dt * b->invI * b->torque;

        // b->velocity += dt * (gravity + b->invMass * b->force);
        // b->angularVelocity += dt * b->invI * b->torque;
	}

    // perform pre-steps
    {
        for (auto& arb : arbiters) arb.second.PreStep(dti);
        for (auto& joint : joints) joint->PreStep(dti);
    }
    // perform iterations
	for (int i = 0; i < iterations; i++)
	{
        for (auto& arb : arbiters) arb.second.ApplyImpulse();
        for (auto& joint : joints) joint->ApplyImpulse();
	}

	// integrate Velocities
    for (auto& b : bodies)
	{
		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;
		b->force.Set(0.0f, 0.0f);
		b->torque = 0.0f;
	}
}
