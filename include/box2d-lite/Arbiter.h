#pragma once

#include "MathUtils.h"

struct Body;

enum EdgeNumbers
{
    NO_EDGE,
    EDGE1,
    EDGE2,
    EDGE3,
    EDGE4
};
struct Edges
{
    char edge1in;
    char edge1out;
    char edge2in;
    char edge2out;
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
