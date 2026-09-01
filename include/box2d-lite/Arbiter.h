#pragma once

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
    void PreStep(float dti)
    {
        for (int i = 0; i < numContacts; i++)
        {
            Contact* c = contacts + i;

            Vec2 normal = c->normal;
            Vec2 tangent = RotateRight(c->normal);
            Vec2 r1 = c->r1;
            Vec2 r2 = c->r2;

            float r1n = Dot(r1, normal);
            float r1t = Dot(r1, tangent);
            float r2n = Dot(r2, normal);
            float r2t = Dot(r2, tangent);
            float r1nl = r1n * r1n;
            float r1tl = r1t * r1t;
            float r2nl = r2n * r2n;
            float r2tl = r2t * r2t;
            float r1l = LengthSqrt(r1);
            float r2l = LengthSqrt(r2);

            float massInvSum = body1->massInv + body2->massInv;

            float massNormal  = massInvSum + body1->inertiaInv * (r1l - r1nl) + body2->inertiaInv * (r2l - r2nl);
            float massTangent = massInvSum + body1->inertiaInv * (r1l - r1tl) + body2->inertiaInv * (r2l - r2tl);

            c->massNormalInv  = 1.0f / massNormal;
            c->massTangentInv = 1.0f / massTangent;

            if (Config::positionCorrection)
            {
                float allowedPenetration = 0.01f;
                float biasFactor = 0.2f;
                c->bias = -Min(c->separation + allowedPenetration, 0.0f) * biasFactor * dti;
            }
            else
            {
                c->bias = 0.0f;
            }

            Vec2 impulse = normal * c->Pn + tangent * c->Pt;
            UpdateVelocity(c, body1, body2, impulse);
        }
    }
    void ApplyImpulse()
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

            float frictionMax = friction * c->Pn;

            {
                Vec2 vr = CalcRelativeVelocity(c, body1, body2);
                Vec2 tangent = RotateRight(c->normal);
                float impInit = -Dot(tangent, vr) * c->massTangentInv;
                float impOld = c->Pt;
                float impNew = Clamp(impOld + impInit, -frictionMax, +frictionMax);
                float impDiff = impNew - impOld;
                Vec2 impulse = tangent * impDiff;
                UpdateVelocity(c, body1, body2, impulse);
                c->Pt = impNew;
            }
        }
    }


private:
    Vec2 CalcRelativeVelocity(Contact* c, Body* b1, Body* b2)
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
    void UpdateVelocity(Contact* c, Body* b1, Body* b2, Vec2 impulse)
    {
        b1->velocityLinear -= impulse * b1->massInv;
        b2->velocityLinear += impulse * b2->massInv;
        b1->velocityAngular -= Cross(c->r1, impulse) * b1->inertiaInv;
        b2->velocityAngular += Cross(c->r2, impulse) * b2->inertiaInv;
    }
};

inline bool operator < (const ArbiterKey& a1, const ArbiterKey& a2)
{
    if (a1.body1 < a2.body1) return true;
    if (a1.body1 > a2.body1) return false;
    if (a1.body2 < a2.body2) return true;
    return false;
}

int Collide(Contact* contacts, const Body* body1, const Body* body2);
