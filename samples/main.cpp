#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#define GLFW_INCLUDE_NONE
#include "glad/glad.h"
#include "GLFW/glfw3.h"

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

#include "box2d-lite/MathUtils.h"

#include "box2d-lite/Body.h"

#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Joint.h"

#include "box2d-lite/World.h"

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

// box schema
//
//   ^ y
//   |
//   + --> x
//
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4
//    |        |
//   v3 ------ v4
//        e3

enum Axis
{
    FACE_A_X,
    FACE_A_Y,
    FACE_B_X,
    FACE_B_Y
};

struct ClipVertex
{
    Vec2 v;
    FeaturePair fp;
};

static void ComputeIncidentEdge(const Body* body, Vec2 normal, ClipVertex& v0, ClipVertex& v1)
{
    Vec2 pos = body->position;
    Vec2 scaleh = body->scale * 0.5f;
    Mat22 rot = Mat22(body->rotation);

    normal = rot.Transpose() * normal;

    if (Abs(normal.x) > Abs(normal.y))
    {
        if (normal.x >= 0.0f)
        {
            v0.v = { +scaleh.x, -scaleh.y }; v0.fp.e.edge2in = EDGE3; v0.fp.e.edge2out = EDGE4;
            v1.v = { +scaleh.x, +scaleh.y }; v1.fp.e.edge2in = EDGE4; v1.fp.e.edge2out = EDGE1;
        }
        else
        {
            v0.v = { -scaleh.x, +scaleh.y }; v0.fp.e.edge2in = EDGE1; v0.fp.e.edge2out = EDGE2;
            v1.v = { -scaleh.x, -scaleh.y }; v1.fp.e.edge2in = EDGE2; v1.fp.e.edge2out = EDGE3;
        }
    }
    else
    {
        if (normal.y >= 0.0f)
        {
            v0.v = { +scaleh.x, +scaleh.y }; v0.fp.e.edge2in = EDGE4; v0.fp.e.edge2out = EDGE1;
            v1.v = { -scaleh.x, +scaleh.y }; v1.fp.e.edge2in = EDGE1; v1.fp.e.edge2out = EDGE2;
        }
        else
        {
            v0.v = { -scaleh.x, -scaleh.y }; v0.fp.e.edge2in = EDGE2; v0.fp.e.edge2out = EDGE3;
            v1.v = { +scaleh.x, -scaleh.y }; v1.fp.e.edge2in = EDGE3; v1.fp.e.edge2out = EDGE4;
        }
    }

    v0.v = pos + rot * v0.v;
    v1.v = pos + rot * v1.v;
}
static bool ClipLine(ClipVertex vIn[MAX_POINTS], ClipVertex vOut[MAX_POINTS], Vec2 normal, float offset, char clipEdge)
{
    // Calculate the distance of end points to the line
    float dist0 = Dot(normal, vIn[0].v) - offset;
    float dist1 = Dot(normal, vIn[1].v) - offset;

    int state = 0;
    if (dist0 < 0.0f) state += 1;
    if (dist1 < 0.0f) state += 2;

    switch (state)
    {
        case 1:
        {
            vOut[0] = vIn[0];
            vOut[1] = vIn[1];
            vOut[1].fp.e.edge1out = clipEdge;
            vOut[1].fp.e.edge2out = NO_EDGE;
            vOut[1].v = Lerp(vIn[0].v, vIn[1].v, dist0 / (dist0 - dist1));
            return false;
        }
        case 2:
        {
            vOut[0] = vIn[1];
            vOut[1] = vIn[0];
            vOut[1].fp.e.edge1in = clipEdge;
            vOut[1].fp.e.edge2in = NO_EDGE;
            vOut[1].v = Lerp(vIn[0].v, vIn[1].v, dist0 / (dist0 - dist1));
            return false;
            // vOut[0] = vIn[0];
            // vOut[1] = vIn[1];
            // vOut[0].fp.e.edge1in = clipEdge;
            // vOut[0].fp.e.edge2in = NO_EDGE;
            // vOut[0].v = Lerp(vIn[0].v, vIn[1].v, dist0 / (dist0 - dist1));
            // return false;
        }
        case 3:
        {
            vOut[0] = vIn[0];
            vOut[1] = vIn[1];
            return false;
        }
        default: return true;
    }
}
static bool Sat(const Body* body1, const Body* body2, Vec2& normal, float& dist, Axis& axis)
{
    Vec2 pos1 = body1->position;
    Vec2 pos2 = body2->position;
    Vec2 scale1 = body1->scale * 0.5f;
    Vec2 scale2 = body2->scale * 0.5f;
    Mat22 rot1 = Mat22(body1->rotation);
    Mat22 rot2 = Mat22(body2->rotation);
    Mat22 rot1i = rot1.Transpose();
    Mat22 rot2i = rot2.Transpose();
    Vec2 d1 = rot1i * (pos2 - pos1);
    Vec2 d2 = rot2i * (pos2 - pos1);
    Mat22 rotc = Abs(rot1i * rot2);
    Mat22 rotci = rotc.Transpose();
    Vec2 face1 = Abs(d1) - scale1 - rotc  * scale2;
    Vec2 face2 = Abs(d2) - scale2 - rotci * scale1;

    if (face1.x > 0.0f) return false;
    if (face1.y > 0.0f) return false;
    if (face2.x > 0.0f) return false;
    if (face2.y > 0.0f) return false;

    // todo double check tr ta
    // tr makes axis switch if diff significant
    // ta makes axis switch to for smaller body

    const float tr = 0.95f; // tolerance relative
    const float ta = 0.01f; // tolerance absolute

                                               { dist = face1.x; axis = FACE_A_X; }
    if ((dist * tr + scale1.y * ta) < face1.y) { dist = face1.y; axis = FACE_A_Y; }
    if ((dist * tr + scale2.x * ta) < face2.x) { dist = face2.x; axis = FACE_B_X; }
    if ((dist * tr + scale2.y * ta) < face2.y) { dist = face2.y; axis = FACE_B_Y; }

    switch (axis)
    {
        case FACE_A_X: normal = d1.x > 0.0f ? rot1.col1 : -rot1.col1; break;
        case FACE_A_Y: normal = d1.y > 0.0f ? rot1.col2 : -rot1.col2; break;
        case FACE_B_X: normal = d2.x > 0.0f ? rot2.col1 : -rot2.col1; break;
        case FACE_B_Y: normal = d2.y > 0.0f ? rot2.col2 : -rot2.col2; break;
    }

    return true;
}

int Collide(Contact* contacts, const Body* body1, const Body* body2)
{
    Vec2 pos1 = body1->position;
    Vec2 pos2 = body2->position;
    Vec2 scaleh1 = body1->scale * 0.5f;
    Vec2 scaleh2 = body2->scale * 0.5f;
    Mat22 rot1 = Mat22(body1->rotation);
    Mat22 rot2 = Mat22(body2->rotation);

    Vec2 normal; float dist; Axis axis;
    auto hit = Sat(body1, body2, normal, dist, axis);
    if (!hit) return 0;

    ClipVertex clipPoints0[MAX_POINTS] = {};
    ClipVertex clipPoints1[MAX_POINTS] = {};
    ClipVertex clipPoints2[MAX_POINTS] = {};

    Vec2 normalFront, normalSide;
    float front, sideNeg, sidePos;
    char edgeNeg, edgePos;

    switch (axis)
    {
        case FACE_A_X:
        {
            ComputeIncidentEdge(body2, -normal, clipPoints0[0], clipPoints0[1]);
            normalFront = normal;
            normalSide = rot1.col2;
            front   = scaleh1.x + Dot(pos1, normalFront);
            sidePos = scaleh1.y + Dot(pos1, normalSide);
            sideNeg = scaleh1.y - Dot(pos1, normalSide);
            edgePos = EDGE1;
            edgeNeg = EDGE3;
        }
        break;

        case FACE_A_Y:
        {
            ComputeIncidentEdge(body2, -normal, clipPoints0[0], clipPoints0[1]);
            normalFront = normal;
            normalSide = rot1.col1;
            front   = scaleh1.y + Dot(pos1, normalFront);
            sidePos = scaleh1.x + Dot(pos1, normalSide);
            sideNeg = scaleh1.x - Dot(pos1, normalSide);
            edgePos = EDGE4;
            edgeNeg = EDGE2;
        }
        break;

        case FACE_B_X:
        {
            ComputeIncidentEdge(body1, normal, clipPoints0[0], clipPoints0[1]);
            normalFront = -normal;
            normalSide = rot2.col2;
            front   = scaleh2.x + Dot(pos2, normalFront);
            sidePos = scaleh2.y + Dot(pos2, normalSide);
            sideNeg = scaleh2.y - Dot(pos2, normalSide);
            edgePos = EDGE1;
            edgeNeg = EDGE3;
        }
        break;

        case FACE_B_Y:
        {
            ComputeIncidentEdge(body1, normal, clipPoints0[0], clipPoints0[1]);
            normalFront = -normal;
            normalSide = rot2.col1;
            front   = scaleh2.y + Dot(pos2, normalFront);
            sidePos = scaleh2.x + Dot(pos2, normalSide);
            sideNeg = scaleh2.x - Dot(pos2, normalSide);
            edgePos = EDGE4;
            edgeNeg = EDGE2;
        }
        break;
    }

    if (ClipLine(clipPoints0, clipPoints1, +normalSide, sidePos, edgePos)) return 0;
    if (ClipLine(clipPoints1, clipPoints2, -normalSide, sideNeg, edgeNeg)) return 0;

    // removes points ouside referance box

    int numContacts = 0;

    for (int i = 0; i < MAX_POINTS; i++)
    {
        auto& point = clipPoints2[i];

        float separation = Dot(normalFront, point.v) - front;
        if (separation > 0.0f) continue;

        auto& contact = contacts[numContacts];

        // clamp to reference face (easy to cull)
        contact.position = point.v - normalFront * separation;
        contact.feature = point.fp;

        contact.normal = normal;
        contact.separation = separation;

        contact.r1 = contact.position - body1->position;
        contact.r2 = contact.position - body2->position;

        if (axis == FACE_B_X || axis == FACE_B_Y)
        {
            Swap(contact.feature.e.edge1in, contact.feature.e.edge2in);
            Swap(contact.feature.e.edge1out, contact.feature.e.edge2out);
        }

        numContacts++;
    }

    return numContacts;
}


namespace
{
    int width = 1280;
    int height = 720;
    float zoom = 10.0f;
    float pan_y = 8.0f;
    GLFWwindow* window = NULL;

    // int width = 1280;
    // int height = 720;
    // float zoom = 2.0f;
    // float pan_y = 0.0f;
    // GLFWwindow* window = NULL;

    float timestep = 1.0f / 60.0f;
    bool pause = false;
    bool forward = false;

    int demoIndex = 0;

    Vec2 gravity = { 0.0f, -10.0f };
    int iterations = 10;
    Body body_s[200];
    Joint joint_s[100];
    int body_s_count = 0;
    int joint_s_count = 0;
    Body* bomb = NULL;

    int closeBodyIndex = -1;
    Vec2 closeBodyPoint;
    Vec2 closeBodyOffset;

    int selectedBodyIndex = -1;
    Vec2 selectedBodyPoint;

    World world(gravity, iterations);
}

void LaunchBomb()
{
    if (!bomb)
    {
        bomb = body_s + body_s_count;
        bomb->Set(Vec2(1.0f, 1.0f), 50.0f);
        bomb->friction = 0.2f;
        world.Add(bomb);
        body_s_count++;
    }

    bomb->position.Set(Random(-15.0f, 15.0f), 15.0f);
    bomb->rotation = Random(-1.5f, 1.5f);
    bomb->velocityLinear = -1.5f * bomb->position;
    bomb->velocityAngular = Random(-20.0f, 20.0f);
}

void AddGround(Body* b)
{
    b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
    b->position = { 0.0f, b->scale.y * -0.5f };
    world.Add(b);
}
void AddBox(Vec2 coord)
{
    auto b = &body_s[body_s_count];
    // b->Set(Vec2(1.0f, 1.0f), 200.0f);
    b->Set(Vec2(1.0f, 1.0f), 10.0f);
    b->position = coord;
    world.Add(b);
    body_s_count++;
}

void Demo1(Body* b, Joint* j)
{
    // AddGround(b);
    // b++; body_s_count++;

    // b->Set({ 1.0f, 1.0f }, 1.0f);
    // b->position.Set(0.0f, 4.0f);
    // world.Add(b);
    // b++; body_s_count++;



    b->Set({ 1.0f, 1.0f }, FLT_MAX);
    b->position = { 0.0f, 0.0f };
    world.Add(b);
    b++; body_s_count++;

    b->Set({ 0.5f, 0.5f }, 1.0f);
    // b->position = { 0.60f, 0.0f };
    b->position = { -0.60f, 0.0f };
    b->rotation = -MATH_PI/4;
    // b->rotation = -0.3f;
    world.Add(b);
    b++; body_s_count++;
}
void Demo2(Body* b, Joint* j)
{
    auto b1 = b;
    AddGround(b);
    b++; body_s_count++;

    auto b2 = b;
    b2->Set(Vec2(1.0f, 1.0f), 100.0f);
    b2->friction = 0.2f;
    b2->position.Set(9.0f, 11.0f);
    b2->rotation = 0.0f;
    world.Add(b2);
    b++; body_s_count++;

    j->Set(b1, b2, Vec2(0.0f, 11.0f));
    world.Add(j);
    joint_s_count++;
}
void Demo3(Body* b, Joint* j)
{
    AddGround(b);
    ++b; ++body_s_count;

    b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
    b->position.Set(-2.0f, 11.0f);
    b->rotation = -0.25f;
    world.Add(b);
    ++b; ++body_s_count;

    b->Set(Vec2(0.25f, 1.0f), FLT_MAX);
    b->position.Set(5.25f, 9.5f);
    world.Add(b);
    ++b; ++body_s_count;

    b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
    b->position.Set(2.0f, 7.0f);
    b->rotation = 0.25f;
    world.Add(b);
    ++b; ++body_s_count;

    b->Set(Vec2(0.25f, 1.0f), FLT_MAX);
    b->position.Set(-5.25f, 5.5f);
    world.Add(b);
    ++b; ++body_s_count;

    b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
    b->position.Set(-2.0f, 3.0f);
    b->rotation = -0.25f;
    world.Add(b);
    ++b; ++body_s_count;

    float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
    for (int i = 0; i < 5; ++i)
    {
        b->Set(Vec2(0.5f, 0.5f), 25.0f);
        b->friction = friction[i];
        b->position.Set(-7.5f + 2.0f * i, 14.0f);
        world.Add(b);
        ++b; ++body_s_count;
    }

    // b->Set(Vec2(0.5f, 0.5f), 25.0f);
    // b->friction = 100.75f;
    // b->position.Set(-7.5f + 2.0f, 14.0f);
    // world.Add(b);
    // ++b; ++body_s_count;
}
void Demo4(Body* b, Joint* j)
{
    AddGround(b);
    ++b; ++body_s_count;

    for (int i = 0; i < 10; ++i)
    {
        b->Set(Vec2(1.0f, 1.0f), 1.0f);
        b->friction = 0.2f;
        float x = Random(-0.1f, 0.1f);
        b->position.Set(x, 0.51f + 1.05f * i);
        world.Add(b);
        ++b; ++body_s_count;
    }
}
void Demo5(Body* b, Joint* j)
{
    AddGround(b);
    ++b; ++body_s_count;

    Vec2 x = { -6.0f, 0.75f };

    for (int i = 0; i < 12; ++i)
    {
        Vec2 y = x;

        for (int j = i; j < 12; j++)
        {
            b->Set(Vec2(1.0f, 1.0f), 10.0f);
            b->friction = 0.2f;
            b->position = y;
            world.Add(b);
            ++b; ++body_s_count;

            y += Vec2(1.125f, 0.0f);
        }

        x += Vec2(0.5625f, 2.0f);
    }
}
void Demo6(Body* b, Joint* j)
{
    Body* b1 = b;
    AddGround(b);
    ++b; ++body_s_count;

    Body* b2 = b;
    b2->Set(Vec2(12.0f, 0.25f), 100.0f);
    b2->position.Set(0.0f, 1.0f);
    world.Add(b2);
    ++b; ++body_s_count;

    Body* b3 = b;
    b3->Set(Vec2(0.5f, 0.5f), 25.0f);
    b3->position.Set(-5.0f, 2.0f);
    world.Add(b3);
    ++b; ++body_s_count;

    Body* b4 = b;
    b4->Set(Vec2(0.5f, 0.5f), 25.0f);
    b4->position.Set(-5.5f, 2.0f);
    world.Add(b4);
    ++b; ++body_s_count;

    Body* b5 = b;
    b5->Set(Vec2(1.0f, 1.0f), 100.0f);
    b5->position.Set(5.5f, 15.0f);
    world.Add(b5);
    ++b; ++body_s_count;

    j->Set(b1, b2, Vec2(0.0f, 1.0f));
    world.Add(j);

    joint_s_count += 1;
}
void Demo7(Body* b, Joint* j)
{
    AddGround(b);
    ++b; ++body_s_count;

    const int numPlanks = 15;
    float mass = 50.0f;

    for (int i = 0; i < numPlanks; ++i)
    {
        b->Set(Vec2(1.0f, 0.25f), mass);
        b->friction = 0.2f;
        b->position.Set(-8.5f + 1.25f * i, 5.0f);
        world.Add(b);
        ++b; ++body_s_count;
    }

    // Tuning
    float frequencyHz = 2.0f;
    float dampingRatio = 0.7f;

    // frequency in radians
    float omega = 2.0f * k_pi * frequencyHz;

    // damping coefficient
    float d = 2.0f * mass * dampingRatio * omega;

    // spring stifness
    float k = mass * omega * omega;

    // magic formulas
    float softness = 1.0f / (d + timestep * k);
    float biasFactor = timestep * k / (d + timestep * k);

    for (int i = 0; i < numPlanks; ++i)
    {
        j->Set(body_s+i, body_s+i+1, Vec2(-9.125f + 1.25f * i, 5.0f));
        j->softness = softness;
        j->biasFactor = biasFactor;

        world.Add(j);
        ++j; ++joint_s_count;
    }

    j->Set(body_s + numPlanks, body_s, Vec2(-9.125f + 1.25f * numPlanks, 5.0f));
    j->softness = softness;
    j->biasFactor = biasFactor;
    world.Add(j);
    ++j; ++joint_s_count;
}
void Demo8(Body* b, Joint* j)
{
    Body* b1 = b;
    AddGround(b);
    ++b; ++body_s_count;

    b->Set(Vec2(12.0f, 0.5f), FLT_MAX);
    b->position.Set(-1.5f, 10.0f);
    world.Add(b);
    ++b; ++body_s_count;

    for (int i = 0; i < 10; ++i)
    {
        b->Set(Vec2(0.2f, 2.0f), 10.0f);
        b->position.Set(-6.0f + 1.0f * i, 11.125f);
        b->friction = 0.1f;
        world.Add(b);
        ++b; ++body_s_count;
    }

    b->Set(Vec2(14.0f, 0.5f), FLT_MAX);
    b->position.Set(1.0f, 6.0f);
    b->rotation = 0.3f;
    world.Add(b);
    ++b; ++body_s_count;

    Body* b2 = b;
    b->Set(Vec2(0.5f, 3.0f), FLT_MAX);
    b->position.Set(-7.0f, 4.0f);
    world.Add(b);
    ++b; ++body_s_count;

    Body* b3 = b;
    b->Set(Vec2(12.0f, 0.25f), 20.0f);
    b->position.Set(-0.9f, 1.0f);
    world.Add(b);
    ++b; ++body_s_count;

    j->Set(b1, b3, Vec2(-2.0f, 1.0f));
    world.Add(j);
    ++j; ++joint_s_count;

    Body* b4 = b;
    b->Set(Vec2(0.5f, 0.5f), 10.0f);
    b->position.Set(-10.0f, 15.0f);
    world.Add(b);
    ++b; ++body_s_count;

    j->Set(b2, b4, Vec2(-7.0f, 15.0f));
    world.Add(j);
    ++j; ++joint_s_count;

    Body* b5 = b;
    b->Set(Vec2(2.0f, 2.0f), 20.0f);
    b->position.Set(6.0f, 2.5f);
    b->friction = 0.1f;
    world.Add(b);
    ++b; ++body_s_count;

    j->Set(b1, b5, Vec2(6.0f, 2.6f));
    world.Add(j);
    ++j; ++joint_s_count;

    Body* b6 = b;
    b->Set(Vec2(2.0f, 0.2f), 10.0f);
    b->position.Set(6.0f, 3.6f);
    world.Add(b);
    ++b; ++body_s_count;

    j->Set(b5, b6, Vec2(7.0f, 3.5f));
    world.Add(j);
    ++j; ++joint_s_count;
}
void Demo9(Body* b, Joint* j)
{
    Body* b1 = b;
    AddGround(b);
    ++b; ++body_s_count;

    float mass = 10.0f;

    // Tuning
    float frequencyHz = 4.0f;
    float dampingRatio = 0.7f;

    // frequency in radians
    float omega = 2.0f * k_pi * frequencyHz;

    // damping coefficient
    float d = 2.0f * mass * dampingRatio * omega;

    // spring stiffness
    float k = mass * omega * omega;

    // magic formulas
    float softness = 1.0f / (d + timestep * k);
    float biasFactor = timestep * k / (d + timestep * k);

    const float y = 12.0f;

    for (int i = 0; i < 15; ++i)
    {
        Vec2 x(0.5f + i, y);
        b->Set(Vec2(0.75f, 0.25f), mass);
        b->friction = 0.2f;
        b->position = x;
        b->rotation = 0.0f;
        world.Add(b);

        j->Set(b1, b, Vec2(float(i), y));
        j->softness = softness;
        j->biasFactor = biasFactor;
        world.Add(j);

        b1 = b;
        ++b;
        ++body_s_count;
        ++j;
        ++joint_s_count;
    }
}

const char* demoNames[] =
{
    "Demo 1: A Single Box",
    "Demo 2: Simple Pendulum",
    "Demo 3: Varying Friction Coefficients",
    "Demo 4: Randomized Stacking",
    "Demo 5: Pyramid Stacking",
    "Demo 6: A Teeter",
    "Demo 7: A Suspension Bridge",
    "Demo 8: Dominos",
    "Demo 9: Multi-pendulum"
};
void (*demos[])(Body* b, Joint* j) =
{
    Demo1,
    Demo2,
    Demo3,
    Demo4,
    Demo5,
    Demo6,
    Demo7,
    Demo8,
    Demo9
};

void InitDemo(int index)
{
    world.Clear();
    body_s_count = 0;
    joint_s_count = 0;
    bomb = NULL;

    demoIndex = index;
    demos[index](body_s, joint_s);
}

void SelectBody(Vec2 mousePos)
{
    int index = -1;

    Vec2 offset0;
    float offset0_ls = FLT_MAX;

    for (size_t i = 0; i < world.bodies.size(); i++)
    {
        auto body = world.bodies[i];

        if (body->mass == FLT_MAX) continue;

        Vec2 offset1 = ShortPathToSurface(mousePos, body->position, body->rotation, body->scale);
        float offset1_ls = Dot(offset1, offset1);

        if (offset0_ls <= offset1_ls) continue;

        index = i;

        offset0 = offset1;
        offset0_ls = offset1_ls;
    }

    if (index == -1) PANIC

    closeBodyIndex = index;
    closeBodyOffset = offset0;
    closeBodyPoint = mousePos + offset0;
}

Vec2 ScreenToWorld(float x, float y)
{
    Vec2 result;

    Vec2 ndc;
    ndc.x = x / width;
    ndc.y = y / height;
    ndc.y = 1.0f - ndc.y;
    ndc.x = ndc.x * 2.0f - 1.0f;
    ndc.y = ndc.y * 2.0f - 1.0f;

    if (width > height)
    {
        float aspect = float(width) / float(height);
        result.x = ndc.x * aspect;
        result.y = ndc.y;
    }
    else
    {
        float aspect = float(height) / float(width);
        result.x = ndc.x;
        result.y = ndc.y * aspect;
    }

    result.x *= zoom;
    result.y *= zoom;

    result.y += pan_y;

    return result;
}
Vec2 GetMousePosition()
{
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    return ScreenToWorld(xpos, ypos);
}
void ErrorCallback(int error, const char* description)
{
    printf("GLFW error %d: %s\n", error, description);
}
void SetProj()
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = float(width) / float(height);
    if (width >= height)
    {
        // aspect >= 1, set the height from -1 to 1, with larger width
        glOrtho(-zoom * aspect, zoom * aspect, -zoom + pan_y, zoom + pan_y, -1.0, 1.0);
    }
    else
    {
        // aspect < 1, set the width to -1 to 1, with larger height
        glOrtho(-zoom, zoom, -zoom / aspect + pan_y, zoom / aspect + pan_y, -1.0, 1.0);
    }
}
void Reshape(GLFWwindow*, int w, int h)
{
    width = w;
    height = h > 0 ? h : 1;
    SetProj();
}
void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS) return;

    switch (key)
    {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;

        case GLFW_KEY_P:
            pause = !pause;
            break;

        case GLFW_KEY_RIGHT_BRACKET:
            forward = true;
            break;

        case GLFW_KEY_A:
            World::accumulateImpulses = !World::accumulateImpulses;
            break;

        case GLFW_KEY_S:
            World::positionCorrection = !World::positionCorrection;
            break;

        case GLFW_KEY_D:
            World::warmStarting = !World::warmStarting;
            break;

        case GLFW_KEY_SPACE:
            LaunchBomb();
            break;

        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            InitDemo(key - GLFW_KEY_1);
            break;
    }
}
void Mouse(GLFWwindow* window, int button, int action, int mods)
{
    if (action != GLFW_PRESS) return;
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    auto mousePosition = GetMousePosition();

    // AddBox(coord);

    if (closeBodyIndex == -1) return;

    if (selectedBodyIndex == -1)
    {
        selectedBodyPoint = closeBodyPoint;
        selectedBodyIndex = closeBodyIndex;
    }
    else
    {
        auto body = world.bodies[selectedBodyIndex];
        auto point = selectedBodyPoint;
        auto velocity = mousePosition - point;
        world.ApplyImpulse(body, point, velocity);
        selectedBodyIndex = -1;
    }
}
void DrawText(int x, int y, const char* string)
{
    ImVec2 p;
    p.x = float(x);
    p.y = float(y);
    ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPos(p);
    ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
    ImGui::End();
}
void DrawPoint(Vec2 p)
{
    glPointSize(4.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    glVertex2f(p.x, p.y);
    glEnd();
    glPointSize(1.0f);
}
void DrawLine(Vec2 p0, Vec2 p1)
{
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex2f(p0.x, p0.y);
    glVertex2f(p1.x, p1.y);
    glEnd();
}
void DrawBody(Body* body, bool selected)
{
    Mat22 R(body->rotation);
    Vec2 p = body->position;
    Vec2 h = 0.5f * body->scale;

    Vec2 v1 = p + R * Vec2(-h.x, -h.y);
    Vec2 v2 = p + R * Vec2(+h.x, -h.y);
    Vec2 v3 = p + R * Vec2(+h.x, +h.y);
    Vec2 v4 = p + R * Vec2(-h.x, +h.y);

    if (selected)          glColor3f(1.0f, 0.0f, 0.0f);
    else if (body == bomb) glColor3f(0.4f, 0.9f, 0.4f);
    else                   glColor3f(0.8f, 0.8f, 0.9f);

    glBegin(GL_LINE_LOOP);
    glVertex2f(v1.x, v1.y);
    glVertex2f(v2.x, v2.y);
    glVertex2f(v3.x, v3.y);
    glVertex2f(v4.x, v4.y);
    glEnd();
}
void DrawJoint(Joint* joint)
{
    Body* b1 = joint->body1;
    Body* b2 = joint->body2;

    Mat22 R1(b1->rotation);
    Mat22 R2(b2->rotation);

    Vec2 x1 = b1->position;
    Vec2 p1 = x1 + R1 * joint->localAnchor1;

    Vec2 x2 = b2->position;
    Vec2 p2 = x2 + R2 * joint->localAnchor2;

    glColor3f(0.5f, 0.5f, 0.8f);
    glBegin(GL_LINES);
    glVertex2f(x1.x, x1.y);
    glVertex2f(p1.x, p1.y);
    glVertex2f(x2.x, x2.y);
    glVertex2f(p2.x, p2.y);
    glEnd();
}
void DrawArbiter(Arbiter* arbiter)
{
    glPointSize(4.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);

    for (int i = 0; i < arbiter->numContacts; i++)
    {
        Vec2 p = arbiter->contacts[i].position;
        glVertex2f(p.x, p.y);
    }

    glEnd();
    glPointSize(1.0f);
}
void Draw()
{
    auto mousePos = GetMousePosition();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Globally position text
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));
    ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::End();

    DrawText(5, 5, demoNames[demoIndex]);
    DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

    char buffer[64];
    sprintf(buffer, "(A) Accumulation %s",        World::accumulateImpulses ? "ON" : "OFF"); DrawText(5, 65,  buffer);
    sprintf(buffer, "(S) Position Correction %s", World::positionCorrection ? "ON" : "OFF"); DrawText(5, 95,  buffer);
    sprintf(buffer, "(D) Warm Starting %s",       World::warmStarting       ? "ON" : "OFF"); DrawText(5, 125, buffer);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // if (selectedBodyIndex == -1)
    // {
    //     if (closeBodyIndex != -1)
    //         DrawPoint(closeBodyPoint);
    // }
    // else
    // {
    //     DrawPoint(selectedBodyPoint);
    //     DrawLine(selectedBodyPoint, mousePos);
    // }

    for (int i = 0; i < body_s_count; i++)
        DrawBody(body_s + i, false);

    for (int i = 0; i < joint_s_count; i++)
        DrawJoint(joint_s + i);

    for (auto& i : world.arbiters)
        DrawArbiter(&i.second);

    // DrawPoint({ 0.246447, 0.000000 });
    // DrawPoint({ 0.600000, -0.353553 });

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}
void InitWindow()
{
    glfwSetErrorCallback(ErrorCallback);

    if (glfwInit() == 0)
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        PANIC;
    }

    window = glfwCreateWindow(width, height, "box2d-lite", NULL, NULL);
    if (window == NULL)
    {
        fprintf(stderr, "Failed to open GLFW window.\n");
        glfwTerminate();
        PANIC;
    }

    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, Mouse);

    int gladStatus = gladLoadGL();
    if (gladStatus == 0)
    {
        fprintf(stderr, "Failed to load OpenGL.\n");
        glfwTerminate();
        PANIC;
    }

    glfwSwapInterval(1);
    glfwSetWindowSizeCallback(window, Reshape);
    glfwSetKeyCallback(window, Keyboard);

    float xscale, yscale;
    glfwGetWindowContentScale(window, &xscale, &yscale);
    float uiScale = xscale;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsClassic();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();
    ImGuiIO& io = ImGui::GetIO();
    io.FontGlobalScale = uiScale;

    SetProj();
}

int main()
{
    InitWindow();

    // InitDemo(0);
    // world.BroadPhase();
    // pause = true;

    InitDemo(3);

    while (!glfwWindowShouldClose(window))
    {
        auto mousePos = GetMousePosition();

        SelectBody(mousePos);

        auto update = !pause || forward; forward = false;
        if (update)
        {
            world.Step(timestep);
            // world.BroadPhase();
        }

        Draw();

        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    glfwTerminate();

    return 0;
}
