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

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#define GLFW_INCLUDE_NONE
#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

namespace
{
    int width = 1280;
	int height = 720;
	float zoom = 10.0f;
	float pan_y = 8.0f;
	GLFWwindow* window = NULL;

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

	World world(gravity, iterations);
}

static void LaunchBomb()
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

static void AddGround(Body* b)
{
    b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position = { 0.0f, b->width.y * -0.5f };
	world.Add(b);
}

static void Demo1(Body* b, Joint* j)
{
    AddGround(b);
	b++; body_s_count++;

	b->Set(Vec2(1.0f, 1.0f), 200.0f);
	b->position.Set(0.0f, 4.0f);
	world.Add(b);
	b++; body_s_count++;
}
static void Demo2(Body* b, Joint* j)
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
static void Demo3(Body* b, Joint* j)
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
static void Demo4(Body* b, Joint* j)
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
static void Demo5(Body* b, Joint* j)
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
static void Demo6(Body* b, Joint* j)
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
static void Demo7(Body* b, Joint* j)
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
static void Demo8(Body* b, Joint* j)
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
static void Demo9(Body* b, Joint* j)
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

static void InitDemo(int index)
{
	world.Clear();
	body_s_count = 0;
	joint_s_count = 0;
	bomb = NULL;

	demoIndex = index;
	demos[index](body_s, joint_s);
}

static void glfwErrorCallback(int error, const char* description)
{
	printf("GLFW error %d: %s\n", error, description);
}
static void DrawText(int x, int y, const char* string)
{
	ImVec2 p;
	p.x = float(x);
	p.y = float(y);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(p);
	ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
	ImGui::End();
}
static void DrawBody(Body* body)
{
	Mat22 R(body->rotation);
	Vec2 p = body->position;
	Vec2 h = 0.5f * body->width;

	Vec2 v1 = p + R * Vec2(-h.x, -h.y);
	Vec2 v2 = p + R * Vec2(+h.x, -h.y);
	Vec2 v3 = p + R * Vec2(+h.x, +h.y);
	Vec2 v4 = p + R * Vec2(-h.x, +h.y);

	if (body == bomb)
		glColor3f(0.4f, 0.9f, 0.4f);
	else
		glColor3f(0.8f, 0.8f, 0.9f);

	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}
static void DrawJoint(Joint* joint)
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
static void DrawArbiters()
{
    glPointSize(4.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (auto& i : world.arbiters)
    {
        auto& arbiter = i.second;

        for (int i = 0; i < arbiter.numContacts; i++)
        {
            Vec2 p = arbiter.contacts[i].position;
            glVertex2f(p.x, p.y);
        }
    }
    glEnd();
    glPointSize(1.0f);
}
static void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
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
static void SetProj()
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
static void Reshape(GLFWwindow*, int w, int h)
{
	width = w;
	height = h > 0 ? h : 1;
    SetProj();
}
static void Mouse(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        printf("%f, %f\n", xpos, ypos);
    }
}

int main()
{
	glfwSetErrorCallback(glfwErrorCallback);

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	window = glfwCreateWindow(width, height, "box2d-lite", NULL, NULL);
	if (window == NULL)
	{
		fprintf(stderr, "Failed to open GLFW window.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);

    glfwSetMouseButtonCallback(window, Mouse);

	int gladStatus = gladLoadGL();
	if (gladStatus == 0)
	{
		fprintf(stderr, "Failed to load OpenGL.\n");
		glfwTerminate();
		return -1;
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

	// InitDemo(0);
    InitDemo(3);

	while (!glfwWindowShouldClose(window))
	{
        auto update = !pause || forward; forward = false;
        if (update)
            world.Step(timestep);

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
		sprintf(buffer, "(A) Accumulation %s", World::accumulateImpulses ? "ON" : "OFF");
		DrawText(5, 65, buffer);

		sprintf(buffer, "(S) Position Correction %s", World::positionCorrection ? "ON" : "OFF");
		DrawText(5, 95, buffer);

		sprintf(buffer, "(D) Warm Starting %s", World::warmStarting ? "ON" : "OFF");
		DrawText(5, 125, buffer);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

        for (int i = 0; i < body_s_count; i++)
            DrawBody(body_s + i);

        for (int i = 0; i < joint_s_count; i++)
            DrawJoint(joint_s + i);

        DrawArbiters();

		ImGui::Render();
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

		glfwPollEvents();
		glfwSwapBuffers(window);
	}

	glfwTerminate();
	return 0;
}
