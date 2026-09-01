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
        position = { 0.0f, 0.0f };
        rotation = 0.0f;
        scale = { 1.0f, 1.0f };

        velocityLinear = { 0.0f, 0.0f };
        velocityAngular = 0.0f;

        force = { 0.0f, 0.0f };
        torque = 0.0f;

        friction = 0.2f;

        mass = FLT_MAX;
        massInv = 0.0f;
        inertia = FLT_MAX;
        inertiaInv = 0.0f;
    }
};

void BodySet(Body* body, Vec2 scale_, float mass_)
{
    body->position = { 0.0f, 0.0f };
    body->rotation = 0.0f;
    body->scale = scale_;

    body->velocityLinear = { 0.0f, 0.0f };
    body->velocityAngular = 0.0f;

    body->force = { 0.0f, 0.0f };
    body->torque = 0.0f;

    body->friction = 0.2f;

    if (mass_ == FLT_MAX)
    {
        body->mass = FLT_MAX;
        body->massInv = 0.0f;
        body->inertia = FLT_MAX;
        body->inertiaInv = 0.0f;
        return;
    }

    body->mass = mass_;
    body->massInv = 1.0f / body->mass;
    body->inertia = body->mass * (body->scale.x * body->scale.x + body->scale.y * body->scale.y) / 12.0f;
    body->inertiaInv = 1.0f / body->inertia;
}
void BodyAddForce(Body& body, Vec2 force_)
{
    body.force += force_;
}