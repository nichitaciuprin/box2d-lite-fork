/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#include "box2d-lite/Body.h"

Body::Body()
{
    position.Set(0.0f, 0.0f);
    rotation = 0.0f;
    velocityLinear.Set(0.0f, 0.0f);
    velocityAngular = 0.0f;
    force.Set(0.0f, 0.0f);
    torque = 0.0f;
    friction = 0.2f;

    scale.Set(1.0f, 1.0f);
    mass = FLT_MAX;
    massInv = 0.0f;
    inertia = FLT_MAX;
    inertiaInv = 0.0f;
}

void Body::Set(const Vec2& w, float m)
{
    position.Set(0.0f, 0.0f);
    rotation = 0.0f;
    velocityLinear.Set(0.0f, 0.0f);
    velocityAngular = 0.0f;
    force.Set(0.0f, 0.0f);
    torque = 0.0f;
    friction = 0.2f;

    scale = w;
    mass = m;

    if (mass < FLT_MAX)
    {
        massInv = 1.0f / mass;
        inertia = mass * (scale.x * scale.x + scale.y * scale.y) / 12.0f;
        inertiaInv = 1.0f / inertia;
    }
    else
    {
        massInv = 0.0f;
        inertia = FLT_MAX;
        inertiaInv = 0.0f;
    }
}
