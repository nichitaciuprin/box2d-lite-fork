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

