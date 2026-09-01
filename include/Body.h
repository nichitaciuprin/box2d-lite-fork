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
    float mass;
    float massInv;
    float inertia;
    float inertiaInv;
};

Body BodyCreate(Vec2 scale_, float mass_)
{
    Body body;

    body.position = { 0.0f, 0.0f };
    body.rotation = 0.0f;
    body.scale = scale_;

    body.velocityLinear = { 0.0f, 0.0f };
    body.velocityAngular = 0.0f;

    body.force = { 0.0f, 0.0f };
    body.torque = 0.0f;

    body.friction = 0.2f;

    if (mass_ == FLT_MAX)
    {
        body.mass = FLT_MAX;
        body.massInv = 0.0f;
        body.inertia = FLT_MAX;
        body.inertiaInv = 0.0f;
    }
    else
    {
        body.mass = mass_;
        body.massInv = 1.0f / body.mass;
        body.inertia = body.mass * (body.scale.x * body.scale.x + body.scale.y * body.scale.y) / 12.0f;
        body.inertiaInv = 1.0f / body.inertia;
    }

    return body;
}
void BodyAddForce(Body& body, Vec2 force_)
{
    body.force += force_;
}