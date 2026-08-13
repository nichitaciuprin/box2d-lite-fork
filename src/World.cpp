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

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

using std::vector;
using std::map;
using std::pair;

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
void World::Add(Impulse impulse)
{
	impulse_s.push_back(impulse);
}
void World::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}
void World::SelectBody(Vec2 mousePos)
{
    int index = -1;

    Vec2 offset0;
    float offset0_ls = FLT_MAX;

    for (size_t i = 0; i < bodies.size(); i++)
    {
        auto body = bodies[i];

        if (body->mass == FLT_MAX) continue;

        Vec2 offset1 = ShortPathToSurface(mousePos, body->position, body->rotation, body->scale);
        float offset1_ls = Dot(offset1, offset1);

        if (offset0_ls <= offset1_ls) continue;

        index = i;

        offset0 = offset1;
        offset0_ls = offset1_ls;
    }

    if (index == -1) PANIC

    selectedBodyIndex = index;
    selectedBodyOffset = offset0;
    selectedBodyPoint = mousePos + offset0;
}

void World::Step(float dt)
{
    float dti = dt > 0.0f ? 1.0f / dt : 0.0f;

	// determine overlapping bodies and update contact points
	BroadPhase();

    // for (auto& a : arbiters)
    // for (auto& c : a.second.contacts)
    //     printf("%f\n", c.separation);
    // printf("===========\n");

	// integrate forces
    for (auto& body : bodies)
	{
		if (body->massInv == 0.0f) continue;

        body->velocityLinear += gravity * dt;

        body->velocityLinear  += body->force  * body->massInv    * dt;
		body->velocityAngular += body->torque * body->inertiaInv * dt;

        body->force = { 0.0f, 0.0f };
		body->torque = 0.0f;
	}

    for (auto& body : bodies)
    for (auto& imp : impulse_s)
    {
        auto contactPoint = imp.position;
        auto impulse = imp.velocity;
        Vec2 r = body->position - contactPoint;
        body->velocityLinear  += impulse * body->massInv;
        body->velocityAngular += Cross(impulse, r) * body->inertiaInv;
    }
    impulse_s.clear();

    {
        for (auto& arbiter : arbiters) arbiter.second.PreStep(dti);
        for (auto& joint : joints) joint->PreStep(dti);
    }
	for (int i = 0; i < iterations; i++)
	{
        for (auto& arbiter : arbiters) arbiter.second.ApplyImpulse();
        for (auto& joint : joints) joint->ApplyImpulse();
	}

	// integrate velocities
    for (auto& body : bodies)
	{
        if (body->massInv == 0.0f) continue;

		body->position += body->velocityLinear  * dt;
		body->rotation += body->velocityAngular * dt;
	}
}

void World::BroadPhase()
{
	// O(n^2) broad-phase

	for (int i =   0; i < (int)bodies.size(); i++)
    for (int j = i+1; j < (int)bodies.size(); j++)
	{
		Body* b1 = bodies[i];
        Body* b2 = bodies[j];

        if (b1->massInv == 0.0f && b2->massInv == 0.0f) continue;

        Arbiter newArb(b1, b2);
        ArbiterKey key(b1, b2);

        if (newArb.numContacts == 0)
        {
            arbiters.erase(key);
            continue;
        }

        auto iter = arbiters.find(key);

        if (iter == arbiters.end())
        {
            arbiters.insert(ArbPair(key, newArb));
            continue;
        }

        auto a_old = &iter->second;
        auto a_new = &newArb;

        if (World::warmStarting)
        {
            for (int i = 0; i < a_new->numContacts; i++)
            for (int j = 0; j < a_old->numContacts; j++)
            {
                auto& c_new = a_new->contacts[i];
                auto& c_old = a_old->contacts[j];

                if (c_new.feature.value != c_old.feature.value) continue;

                c_new.Pn = c_old.Pn;
                c_new.Pt = c_old.Pt;

                break;
            }
        }

        *a_old = *a_new;
	}
}
