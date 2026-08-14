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

static inline Vec2 CalcRelativeVelocity(Contact* c, Body* b1, Body* b2)
{
    // return
    // b2->velocity + Cross(b2->velocityAngular, c->r2) -
    // b1->velocity - Cross(b1->velocityAngular, c->r1);

    // return
    // b2->velocityLinear + RotateLeft(c->r2) * b2->velocityAngular -
    // b1->velocityLinear - RotateLeft(c->r1) * b1->velocityAngular;

    Vec2 vel1 = b1->velocityLinear + RotateLeft(c->r1) * b1->velocityAngular;
    Vec2 vel2 = b2->velocityLinear + RotateLeft(c->r2) * b2->velocityAngular;
    return vel2 - vel1;
}
static inline void UpdateVelocity(Contact* c, Body* b1, Body* b2, Vec2 impulse)
{
    b1->velocityLinear -= impulse * b1->massInv;
    b2->velocityLinear += impulse * b2->massInv;
    b1->velocityAngular -= Cross(c->r1, impulse) * b1->inertiaInv;
    b2->velocityAngular += Cross(c->r2, impulse) * b2->inertiaInv;
}

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

void Arbiter::PreStep(float dti)
{
	for (int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i;

        Vec2 normal = c->normal;
		Vec2 tangent = RotateRight(c->normal);
        Vec2 r1 = c->r1;
		Vec2 r2 = c->r2;

        float rls1 = LengthSqrt(r1);
        float rls2 = LengthSqrt(r2);
		float rnl1 = Dot(r1, normal);
		float rnl2 = Dot(r2, normal);
		float rtl1 = Dot(r1, tangent);
		float rtl2 = Dot(r2, tangent);
        float rnls1 = rnl1 * rnl1;
        float rnls2 = rnl2 * rnl2;
        float rtls1 = rtl1 * rtl1;
        float rtls2 = rtl2 * rtl2;

        float massSum = body1->massInv + body2->massInv;

		float massNormal  = massSum + body1->inertiaInv * (rls1 - rnls1) + body2->inertiaInv * (rls2 - rnls2);
		float massTangent = massSum + body1->inertiaInv * (rls1 - rtls1) + body2->inertiaInv * (rls2 - rtls2);

		c->massNormalInv  = 1.0f / massNormal;
		c->massTangentInv = 1.0f / massTangent;

        if (World::positionCorrection)
        {
            float allowedPenetration = 0.01f;
	        float biasFactor = 0.2f;
		    c->bias = -Min(c->separation + allowedPenetration, 0.0f) * biasFactor * dti;
        }
        else
        {
            c->bias = 0.0f;
        }

        // apply normal and friction impulse
        Vec2 impulse = normal * c->Pn + tangent * c->Pt;
        UpdateVelocity(c, body1, body2, impulse);
	}
}
void Arbiter::ApplyImpulse()
{
	for (int i = 0; i < numContacts; i++)
	{
		Contact* c = contacts + i;

        {
            Vec2 vr = CalcRelativeVelocity(c, body1, body2);
            Vec2 normal = c->normal;
            float impInit = (-Dot(normal, vr) + c->bias) * c->massNormalInv;
            float impOld = c->Pn;
            float impNew = Max(impOld + impInit, 0.0f);
            float impDiff = impNew - impOld;
            Vec2 impulse = normal * impDiff;
            UpdateVelocity(c, body1, body2, impulse);
            c->Pn = impNew;
        }

        float maxFriction = friction * c->Pn;

        {
            Vec2 vr = CalcRelativeVelocity(c, body1, body2);
            Vec2 tangent = RotateRight(c->normal);
            float impInit = -Dot(tangent, vr) * c->massTangentInv;
            float impOld = c->Pt;
            float impNew = Clamp(impOld + impInit, -maxFriction, +maxFriction);
            float impDiff = impNew - impOld;
            Vec2 impulse = tangent * impDiff;
            UpdateVelocity(c, body1, body2, impulse);
            c->Pt = impNew;
        }

        // {
        //     Vec2 vel1 = body1->velocityLinear + RotateLeft(c->r1) * body1->velocityAngular;
        // }
	}
}
