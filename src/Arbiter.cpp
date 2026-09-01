#include "box2d-lite/MathUtils.h"
#include "box2d-lite/Config.h"
#include "box2d-lite/Body.h"

#include "box2d-lite/Arbiter.h"

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
