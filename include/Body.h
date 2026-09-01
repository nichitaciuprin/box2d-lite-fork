#pragma once

struct Body
{
    Vec2 position;
    float rotation;
    Vec2 scale;

    Vec2 velocityLinear;
    float velocityAngular;

    Vec2 force;
    float torque;

    float friction;
    float mass, massInv;
    float inertia, inertiaInv;

    Body()
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

    void Set(const Vec2& w, float m)
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
    void AddForce(const Vec2& f)
    {
        force += f;
    }
};
