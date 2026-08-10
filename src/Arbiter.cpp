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

#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/World.h"

// #include <cstdio>

Arbiter::Arbiter(Body* b1, Body* b2)
{
	if (b1 < b2)
	{
		body1 = b1;
		body2 = b2;
	}
	else
	{
		body1 = b2;
		body2 = b1;
	}

	numContacts = Collide(contacts, body1, body2);

	friction = sqrtf(body1->friction * body2->friction);
}

void Arbiter::Update(Contact* newContacts, int numNewContacts)
{
	Contact mergedContacts[2];

	for (int i = 0; i < numNewContacts; ++i)
	{
		Contact* cNew = newContacts + i;

		int k = -1;

		for (int j = 0; j < numContacts; ++j)
		{
			Contact* cOld = contacts + j;

			if (cNew->feature.value == cOld->feature.value)
			{
				k = j;
				break;
			}
		}

		if (k > -1)
		{
			Contact* c = mergedContacts + i;
			Contact* cOld = contacts + k;
			*c = *cNew;

			if (World::warmStarting)
			{
				c->Pn = cOld->Pn;
				c->Pt = cOld->Pt;
				c->Pnb = cOld->Pnb;
			}
			else
			{
				c->Pn = 0.0f;
				c->Pt = 0.0f;
				c->Pnb = 0.0f;
			}
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}

	for (int i = 0; i < numNewContacts; i++)
		contacts[i] = mergedContacts[i];

	numContacts = numNewContacts;
}

static inline Vec2 CalcRelativeVelocity(Body* b1, Body* b2, Vec2 r1, Vec2 r2)
{
    return
    b2->velocity + Cross(b2->angularVelocity, r2) -
    b1->velocity - Cross(b1->angularVelocity, r1);
}
static inline void ApplyImpulse2(Body* b1, Body* b2, Vec2 r1, Vec2 r2, Vec2 impulse)
{
    b1->velocity -= b1->invMass * impulse;
    b2->velocity += b2->invMass * impulse;
    b1->angularVelocity -= b1->invI * Cross(r1, impulse);
    b2->angularVelocity += b2->invI * Cross(r2, impulse);
}

void Arbiter::PreStep(float dti)
{
	const float k_allowedPenetration = 0.01f;
	const float k_biasFactor = World::positionCorrection ? 0.2f : 0.0f;

	for (int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i;

        Vec2 normal = c->normal;
		Vec2 tangent = Cross(c->normal, 1.0f);
		Vec2 r1 = c->position - body1->position;
		Vec2 r2 = c->position - body2->position;

        float rls1 = Dot(r1, r1);
        float rls2 = Dot(r2, r2);
		float rnl1 = Dot(r1, normal);
		float rnl2 = Dot(r2, normal);
		float rtl1 = Dot(r1, tangent);
		float rtl2 = Dot(r2, tangent);
        float rnls1 = rnl1 * rnl1;
        float rnls2 = rnl2 * rnl2;
        float rtls1 = rtl1 * rtl1;
        float rtls2 = rtl2 * rtl2;
        float massSum = body1->invMass + body2->invMass;
		float massNormal  = massSum + body1->invI * (rls1 - rnls1) + body2->invI * (rls2 - rnls2);
		float massTangent = massSum + body1->invI * (rls1 - rtls1) + body2->invI * (rls2 - rtls2);

		c->massNormal  = 1.0f / massNormal;
		c->massTangent = 1.0f / massTangent;
		c->bias = Min(0.0f, c->separation + k_allowedPenetration) * -k_biasFactor * dti;

        // Apply normal + friction impulse
        Vec2 impulse = c->Pn * normal + c->Pt * tangent;

        body1->velocity -= body1->invMass * impulse;
        body2->velocity += body2->invMass * impulse;
        body1->angularVelocity -= body1->invI * Cross(r1, impulse);
        body2->angularVelocity += body2->invI * Cross(r2, impulse);
	}
}

void Arbiter::ApplyImpulse()
{
	for (int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i;

		Vec2 r1 = c->position - body1->position;
		Vec2 r2 = c->position - body2->position;

        {
            Vec2 vr = CalcRelativeVelocity(body1, body2, r1, r2);
            Vec2 normal = c->normal;
            float impInit = c->massNormal * (-Dot(vr, normal) + c->bias);
            float impOld = c->Pn;
            float impNew = Max(impOld + impInit, 0.0f);
            float impDiff = impNew - impOld;
            ApplyImpulse2(body1, body2, r1, r2, impDiff * normal);
            c->Pn = impNew;
        }

        float maxFriction = friction * c->Pn;

        {
            Vec2 vr = CalcRelativeVelocity(body1, body2, r1, r2);
            Vec2 tangent = Cross(c->normal, 1.0f);
            float impInit = c->massTangent * -Dot(vr, tangent);
            float impOld = c->Pt;
            float impNew = Clamp(impOld + impInit, -maxFriction, +maxFriction);
            float impDiff = impNew - impOld;
            ApplyImpulse2(body1, body2, r1, r2, impDiff * tangent);
            c->Pt = impNew;
        }
	}
}
