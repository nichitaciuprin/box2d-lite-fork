#include "Core.h"

#define TIMESTEP (1.0f / 60.0f)

namespace
{
    int demoIndex = 0;

    Body* bomb = NULL;

    int closeBodyIndex = -1;
    Vec2 closeBodySurPoint;

    int selectedBodyIndex = -1;
    Vec2 selectedBodySurPointLocal;
}

void CalcJointProp(float timestep, float mass, float frequencyHz, float dampingRatio, float& softness, float& biasFactor)
{
    // frequency in radians
    float omega = frequencyHz * MATH_PI * 2.0f;

    // damping coefficient
    float d = omega * dampingRatio * mass * 2.0f;

    // spring stiffness
    float k = mass * omega * omega;

    // magic formulas
    softness =           1.0f / (d + k * timestep);
    biasFactor = k * timestep / (d + k * timestep);
}
void SelectBody(Vec2 mousePos)
{
    int index = -1;

    Vec2 offset0;
    float offset0_ls = FLT_MAX;

    for (size_t i = 0; i < bodie_s.size(); i++)
    {
        auto body = &bodie_s[i];

        if (body->mass == FLT_MAX) continue;

        Vec2 offset1 = ShortPathToSurface(mousePos, body->position, body->rotation, body->scale);
        float offset1_ls = LengthSqr(offset1);

        if (offset0_ls <= offset1_ls) continue;

        index = i;

        offset0 = offset1;
        offset0_ls = offset1_ls;
    }

    if (index == -1) PANIC

    auto body = &bodie_s[index];

    closeBodyIndex = index;
    closeBodySurPoint = mousePos + offset0;
}
void LaunchBomb()
{
    if (!bomb)
    {
        auto bomb_ = BodyCreate({ 1.0f, 1.0f }, 50.0f);
        bodie_s.push_back(bomb_);
        bomb = &bodie_s.back();
    }

    bomb->position = { Random(-15.0f, 15.0f), 15.0f };
    bomb->rotation = Random(-1.5f, 1.5f);
    bomb->velocityLinear = bomb->position * -1.5f;
    bomb->velocityAngular = Random(-20.0f, 20.0f);
}
void AttachAndPull()
{
    if (closeBodyIndex == -1) return;

    if (selectedBodyIndex == -1)
    {
        auto body = &bodie_s[closeBodyIndex];
        selectedBodyIndex = closeBodyIndex;
        selectedBodySurPointLocal = Rotate(closeBodySurPoint - body->position, -body->rotation);
    }
    else
    {
        auto body = &bodie_s[selectedBodyIndex];
        auto p0 = body->position + Rotate(selectedBodySurPointLocal, body->rotation);
        auto p1 = WindowGetMousePositon();
        auto velocity = p1 - p0;
        BodyApplyImpulse(body, p0, velocity);
        selectedBodyIndex = -1;
    }
}

void DrawBody(Body* body, bool selected)
{
    Mat22 r = FromAngle(body->rotation);
    Vec2 p = body->position;
    Vec2 h = body->scale * 0.5f;

    Vec2 v1 = p + r * (Vec2){ -h.x, -h.y };
    Vec2 v2 = p + r * (Vec2){ +h.x, -h.y };
    Vec2 v3 = p + r * (Vec2){ +h.x, +h.y };
    Vec2 v4 = p + r * (Vec2){ -h.x, +h.y };

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
    auto b0 = joint->body1;
    auto b1 = joint->body2;

    auto p0 = b0->position;
    auto p1 = b1->position;

    auto p2 = p0 + FromAngle(b0->rotation) * joint->localAnchor1;
    auto p3 = p1 + FromAngle(b1->rotation) * joint->localAnchor2;

    Vec3 color = { 0.50f, 0.50f, 0.75f };

    DrawLine(p0, p2, color);
    DrawLine(p1, p3, color);
}
void DrawCollision(Collision* collision)
{
    glPointSize(4.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);

    for (int i = 0; i < collision->contact_num; i++)
    {
        Vec2 p = collision->contact_s[i].position;
        glVertex2f(p.x, p.y);
    }

    glEnd();
    glPointSize(1.0f);
}

void Demo1()
{
    CreateGround();
    CreateBoxDynamic({ 0.0f, 4.0f }, 0.0f, { 1.0f, 1.0f }, 1.0f);

    // CreateBoxStatic({ 0.0f, 0.0f }, 0.0f, { 1.0f, 1.0f });
    // CreateBoxDynamic({ +0.50f, 0.0f }, -MATH_PI / 3.0f, { 0.5f, 0.5f }, 1.0f);

    // auto b0 = CreateBoxDynamic({ -0.5f, 8.0f }, 0, { 0.5f, 0.5f }, 1.0f);
    // auto b1 = CreateBoxDynamic({ +0.5f, 6.0f }, 0, { 0.5f, 0.5f }, 1.0f);
    // auto j = CreateJoint(b0, b1, (b0->position + b1->position) / 2);
    // j->softness = 1.0f;
    // j->biasFactor = 0.0f;
}
void Demo2()
{
    CreateGround();

    for (int i = 0; i < 10; i++)
        auto b1 = CreateBoxDynamic({ Random(-0.1f, 0.1f), 0.51f + 1.05f * i }, 0.0f, { 1.0f, 1.0f }, 1.0f);
}
void Demo3()
{
    CreateGround();

    Vec2 x = { -6.0f, 0.75f };

    for (int i = 0; i < 12; i++)
    {
        Vec2 y = x;

        for (int j = i; j < 12; j++)
        {
            auto b1 = CreateBoxDynamic(y, 0.0f, { 1.0f, 1.0f }, 10.0f);

            y += { 1.125f, 0.0f };
        }

        x += { 0.5625f, 2.0f };
    }
}
void Demo4()
{
    CreateGround();
    CreateBoxStatic({ -2.0f, 11.0f }, -0.25f, { 13.0f, 0.25f });
    CreateBoxStatic({ 5.25f, 9.5f },   0.00f, { 0.25f, 1.0f });
    CreateBoxStatic({ 2.0f, 7.0f },   +0.25f, { 13.0f, 0.25f });
    CreateBoxStatic({ -5.25f, 5.5f },  0.00f, { 0.25f, 1.0f });
    CreateBoxStatic({ -2.0f, 3.0f },  -0.25f, { 13.0f, 0.25f });

    float friction[5] = { 0.75f, 0.50f, 0.35f, 0.10f, 0.0f };

    for (int i = 0; i < 5; i++)
    {
        auto b1 = CreateBoxDynamic({ -7.5f + 2.0f * i, 14.0f }, 0.0f, { 0.5f, 0.5f }, 25.0f);
        b1->friction = friction[i];
    }
}
void Demo5()
{
    auto b1 = CreateGround();
    auto b2 = CreateBoxDynamic({ 9.0f, 11.0f }, 0.0f, { 1.0f, 1.0f }, 100.0f);
    CreateJoint(b1, b2, { 0.0f, 11.0f });
}
void Demo6()
{
    float mass = 10.0f;
    float frequencyHz = 4.0f;
    float dampingRatio = 0.7f;

    float softness, biasFactor;
    CalcJointProp(TIMESTEP, mass, frequencyHz, dampingRatio, softness, biasFactor);

    auto b1 = CreateGround();

    for (int i = 0; i < 15; i++)
    {
        float y = 12.0f;

        auto b2 = CreateBoxDynamic({ 0.5f + i, y }, 0.0f, { 0.75f, 0.25f }, mass);

        auto j = CreateJoint(b1, b2, { (float)i, y });
        j->softness = softness;
        j->biasFactor = biasFactor;

        b1 = b2;
    }
}
void Demo7()
{
    float mass = 50.0f;
    float frequencyHz = 2.0f;
    float dampingRatio = 0.7f;

    float softness, biasFactor;
    CalcJointProp(TIMESTEP, mass, frequencyHz, dampingRatio, softness, biasFactor);

    CreateGround();

    int numPlanks = 15;

    for (int i = 0; i < numPlanks; i++)
        CreateBoxDynamic({ -8.5f + 1.25f * i, 5.0f }, 0.0f, { 1.0f, 0.25f }, mass);

    // auto ground = &bodies.bodies[0];
    // auto p1 = &bodies.bodies[1];
    // auto p2 = &bodies.bodies.back();

    // {
    //     auto j1 = CreateJoint(ground, p2, { -9.125f + 1.25f * i, 5.0f });
    //     j1->softness = softness;
    //     j1->biasFactor = biasFactor;
    // }

    for (int i = 0; i < numPlanks; i++)
    {
        auto j1 = CreateJoint(&bodie_s[i], &bodie_s[i+1], { -9.125f + 1.25f * i, 5.0f });
        j1->softness = softness;
        j1->biasFactor = biasFactor;
    }
    {
        auto j1 = CreateJoint(&bodie_s[numPlanks], &bodie_s[0], { -9.125f + 1.25f * numPlanks, 5.0f });
        j1->softness = softness;
        j1->biasFactor = biasFactor;
    }
}
void Demo8()
{
    auto b1 = CreateGround();
    auto b2 = CreateBoxDynamic({ 0.0f, 1.0f }, 0.0f, { 12.0f, 0.25f }, 100.0f);
    CreateBoxDynamic({ -5.0f, 2.0f }, 0.0f, { 0.5f, 0.5f }, 25.0f);
    CreateBoxDynamic({ -5.5f, 2.0f }, 0.0f, { 0.5f, 0.5f }, 25.0f);
    CreateBoxDynamic({ 5.5f, 15.0f }, 0.0f, { 1.0f, 1.0f }, 100.0f);
    CreateJoint(b1, b2, { 0.0f, 1.0f });
}
void Demo9()
{
    auto b1 = CreateGround();

    CreateBoxStatic({ -1.5f, 10.0f }, 0.0f, { 12.0f, 0.5f });
    CreateBoxStatic({ 1.0f, 6.0f }, 0.3f, { 14.0f, 0.5f });

    for (int i = 0; i < 10; i++)
    {
        auto b = CreateBoxDynamic({ -6.0f + 1.0f * i, 11.125f }, 0.0f, { 0.2f, 2.0f }, 10.0f);
        b->friction = 0.1f;
    }

    auto b2 = CreateBoxStatic({ -7.0f, 4.0f }, 0.0f, { 0.5f, 3.0f });
    auto b3 = CreateBoxDynamic({ -0.9f, 1.0f }, 0.0f, { 12.0f, 0.25f }, 20.0f);
    auto b4 = CreateBoxDynamic({ -10.0f, 15.0f }, 0.0f, { 0.5f, 0.5f }, 10.0f);
    auto b5 = CreateBoxDynamic({ 6.0f, 2.5f }, 0.0f, { 2.0f, 2.0f }, 20.0f);
    auto b6 = CreateBoxDynamic({ 6.0f, 3.6f }, 0.0f, { 2.0f, 0.2f }, 10.0f);

    b5->friction = 0.1f;

    CreateJoint(b1, b3, { -2.0f, 1.0f });
    CreateJoint(b2, b4, { -7.0f, 15.0f });
    CreateJoint(b1, b5, { 6.0f, 2.6f });
    CreateJoint(b5, b6, { 7.0f, 3.5f });
}

const char* demoNames[] =
{
    "Demo 1: Single Box",
    "Demo 2: Randomized Stacking",
    "Demo 3: Pyramid Stacking",
    "Demo 4: Varying Friction Coefficients",
    "Demo 5: Simple Pendulum",
    "Demo 6: Multi-pendulum",
    "Demo 7: Suspension Bridge",
    "Demo 8: Teeter",
    "Demo 9: Dominos",
};
void (*demos[])() =
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
    // TODO ref body by index, not pointer, and remove this reserve
    bodie_s.reserve(256);
    joint_s.reserve(256);

    Clear();
    bomb = NULL;
    demoIndex = index;
    demos[index]();
}
void Draw()
{
    ClearScreen();

    if (selectedBodyIndex == -1)
    {
        if (closeBodyIndex != -1)
            DrawPoint(closeBodySurPoint, Color3::GREEN);
    }
    else
    {
        auto body = &bodie_s[selectedBodyIndex];
        auto p0 = body->position + Rotate(selectedBodySurPointLocal, body->rotation);
        auto p1 = WindowGetMousePositon();
        DrawLine(p0, p1, Color3::GREEN);
        DrawPoint(p0, Color3::GREEN);
    }

    for (auto& i : bodie_s) DrawBody(&i, false);
    for (auto& i : joint_s) DrawJoint(&i);
    for (auto& i : collision_s) DrawCollision(&i.second);

    GuiStart();
    {
        ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));

        // ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
        // ImGui::End();

        DrawText(5, 5, demoNames[demoIndex]);
        DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

        char buffer[512];
        sprintf(buffer, "(A) Accumulation %s",        Config::accumulateImpulses ? "ON" : "OFF"); DrawText(5, 65 + 30*0, buffer);
        sprintf(buffer, "(S) Position Correction %s", Config::positionCorrection ? "ON" : "OFF"); DrawText(5, 65 + 30*1, buffer);
        sprintf(buffer, "(D) Warm Starting %s",       Config::warmStarting       ? "ON" : "OFF"); DrawText(5, 65 + 30*2, buffer);
    }
    GuiEnd();
}

int main()
{
    WindowInit();

    InitDemo(0);

    while (!WindowShouldClose())
    {
        if (GetKeyPressed(GLFW_KEY_ESCAPE)) break;

        if (GetKeyPressed(GLFW_KEY_P)) pause = !pause;
        if (GetKeyPressed(GLFW_KEY_RIGHT_BRACKET)) step = true;

        if (GetKeyPressed(GLFW_KEY_A)) Config::accumulateImpulses = !Config::accumulateImpulses;
        if (GetKeyPressed(GLFW_KEY_S)) Config::positionCorrection = !Config::positionCorrection;
        if (GetKeyPressed(GLFW_KEY_D)) Config::warmStarting       = !Config::warmStarting;

        if (GetKeyPressed(GLFW_KEY_SPACE))
            LaunchBomb();

        auto mousePos = WindowGetMousePositon();

        SelectBody(mousePos);

        if (KEY_M0_P)
            AttachAndPull();

        int demoNum = GetNumKeyPressed();
        if (demoNum > 0)
            InitDemo(demoNum-1);

        auto update = !pause || step; step = false;
        if (update)
            Step(TIMESTEP);

        // TODO for points draw
        BroadPhase();

        Draw();

        WindowUpdate();
    }

    WindowClose();

    return 0;
}
