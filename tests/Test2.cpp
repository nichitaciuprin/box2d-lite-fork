#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl2.h"

#include "LangBase.h"
#include "MathUtils.h"
#include "Config.h"
#include "Window.h"
#include "Physics.h"

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
    closeBodySurPoint = Rotate(closeBodySurPoint - body->position, -body->rotation);
}
void AttachAndPull()
{
    if (closeBodyIndex == -1) return;

    if (selectedBodyIndex == -1)
    {
        auto body = &bodie_s[closeBodyIndex];
        selectedBodyIndex = closeBodyIndex;
        selectedBodySurPointLocal = closeBodySurPoint;
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
void LaunchBomb()
{
    if (!bomb)
        bomb = CreateBoxDynamic({}, 0.0f, { 1.0f, 1.0f }, 50.0f);

    bomb->position = { Random(-15.0f, 15.0f), 15.0f };
    bomb->rotation = Random(-1.5f, 1.5f);
    bomb->scale = { 1.0f, 1.0f };
    bomb->velocityLinear = bomb->position * -1.5f;
    bomb->velocityAngular = Random(-20.0f, 20.0f);
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

void InitDemo()
{
    // TODO ref body by index, not pointer, and remove this reserve
    bodie_s.reserve(256);

    Clear();
    bomb = NULL;

    CreateGround();
    CreateBoxDynamic({ 0.0f, 4.0f }, 0.0f, { 1.0f, 1.0f }, 1.0f);
}

void Input()
{
    if (GetKeyPressed(GLFW_KEY_S)) Config::positionCorrection = !Config::positionCorrection;
    if (GetKeyPressed(GLFW_KEY_D)) Config::warmStarting       = !Config::warmStarting;

    if (GetKeyPressed(GLFW_KEY_SPACE))
        LaunchBomb();

    auto mousePos = WindowGetMousePositon();

    SelectBody(mousePos);

    if (KEY_M0_P)
        AttachAndPull();

    if (GetKeyPressed(GLFW_KEY_R))
        InitDemo();
}
void Update()
{
    if (WindowUpdateLoop)
        Step(TIMESTEP);

    // TODO for points draw
    BroadPhase();
}
void Draw()
{
    ClearScreen();

    if (selectedBodyIndex == -1)
    {
        if (closeBodyIndex != -1)
        {
            auto body = &bodie_s[closeBodyIndex];
            auto p0 = body->position + Rotate(closeBodySurPoint, body->rotation);
            DrawPoint(p0, Color3::GREEN);
        }
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
    for (auto& i : collision_s) DrawCollision(&i.second);
}

int main()
{
    WindowInit();

    InitDemo();

    while (!WindowShouldClose())
    {
        if (KEY_ESC) break;

        Input();
        Update();
        Draw();

        WindowUpdate();
    }

    WindowClose();

    return 0;
}
