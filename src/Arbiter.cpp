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

void Arbiter::PreStep(float inv_dt)
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
		c->bias = Min(0.0f, c->separation + k_allowedPenetration) * -k_biasFactor * inv_dt;

		if (!World::accumulateImpulses) continue;

        // Apply normal + friction impulse
        Vec2 P = c->Pn * normal + c->Pt * tangent;

        body1->velocity -= body1->invMass * P;
        body2->velocity += body2->invMass * P;
        body1->angularVelocity -= body1->invI * Cross(r1, P);
        body2->angularVelocity += body2->invI * Cross(r2, P);
	}
}

void Arbiter::ApplyImpulse()
{
	Body* b1 = body1;
	Body* b2 = body2;

	for (int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i;
        // Contact* c = &contacts[i];

		c->r1 = c->position - b1->position;
		c->r2 = c->position - b2->position;

        Vec2 normal = c->normal;
        Vec2 tangent = Cross(c->normal, 1.0f);

        float dPn, dPt;

        {
            // relative velocity at contact
            Vec2 dv =
            b2->velocity + Cross(b2->angularVelocity, c->r2) -
            b1->velocity - Cross(b1->angularVelocity, c->r1);

            dPn = c->massNormal * (-Dot(dv, normal) + c->bias);

            if (World::accumulateImpulses)
            {
                // clamp the accumulated impulse
                float Pn0 = c->Pn;
                c->Pn = Max(Pn0 + dPn, 0.0f);
                dPn = c->Pn - Pn0;
            }
            else
            {
                dPn = Max(dPn, 0.0f);
            }

            // apply contact impulse
            Vec2 Pn = dPn * normal;

            b1->velocity -= b1->invMass * Pn;
		    b2->velocity += b2->invMass * Pn;
            b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);
            b2->angularVelocity += b2->invI * Cross(c->r2, Pn);
        }
        {
            // relative velocity at contact
            Vec2 dv =
            b2->velocity + Cross(b2->angularVelocity, c->r2) -
            b1->velocity - Cross(b1->angularVelocity, c->r1);

            dPt = c->massTangent * (-Dot(dv, tangent));

            if (World::accumulateImpulses)
            {
                // compute friction impulse
                float maxPt = friction * c->Pn;

                // clamp friction
                float oldTangentImpulse = c->Pt;
                c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
                dPt = c->Pt - oldTangentImpulse;
            }
            else
            {
                float maxPt = friction * dPn;
                dPt = Clamp(dPt, -maxPt, maxPt);
            }

            // Apply contact impulse
            Vec2 Pt = dPt * tangent;

            b1->velocity -= b1->invMass * Pt;
            b2->velocity += b2->invMass * Pt;
            b1->angularVelocity -= b1->invI * Cross(c->r1, Pt);
            b2->angularVelocity += b2->invI * Cross(c->r2, Pt);
        }
	}
}
