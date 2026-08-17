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

#ifndef ARBITER_H
#define ARBITER_H

#include "MathUtils.h"

struct Body;

struct Edges
{
    char inEdge1;
    char outEdge1;
    char inEdge2;
    char outEdge2;
};

union FeaturePair
{
	Edges e;
	int value;
};

struct Contact
{
	Contact() : Pn(0.0f), Pt(0.0f) {}

	Vec2 position;
	Vec2 normal;
    Vec2 r1;
    Vec2 r2;
	float separation;
	float Pn;	// accumulated normal impulse
	float Pt;	// accumulated tangent impulse
	float massNormalInv;
    float massTangentInv;
	float bias;
	FeaturePair feature;
};

struct ArbiterKey
{
    Body* body1;
	Body* body2;

	ArbiterKey(Body* b1, Body* b2)
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
	}
};

struct Arbiter
{
public:
    static constexpr int MAX_POINTS = 2;

    Contact contacts[MAX_POINTS];
	int numContacts;
	Body* body1;
	Body* body2;
	float friction; // Combined friction

	Arbiter(Body* b1, Body* b2);
	void PreStep(float inv_dt);
	void ApplyImpulse();
};

// This is used by std::set
inline bool operator < (const ArbiterKey& a1, const ArbiterKey& a2)
{
	if (a1.body1 < a2.body1)
		return true;

	if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
		return true;

	return false;
}

int Collide(Contact* contacts, Body* body1, Body* body2);

#endif
