#pragma once

namespace Color3
{
    inline constexpr Vec3 BLACK      = { 0.0f, 0.0f, 0.0f };
    inline constexpr Vec3 WHITE      = { 1.0f, 1.0f, 1.0f };
    inline constexpr Vec3 RED        = { 1.0f, 0.0f, 0.0f };
    inline constexpr Vec3 GREEN      = { 0.0f, 1.0f, 0.0f };
    inline constexpr Vec3 BLUE       = { 0.0f, 0.0f, 1.0f };
    inline constexpr Vec3 YELLOW     = { 1.0f, 1.0f, 0.0f };
    inline constexpr Vec3 MAGENTA    = { 1.0f, 0.0f, 1.0f };
    inline constexpr Vec3 CYAN       = { 0.0f, 1.0f, 1.0f };
    inline constexpr Vec3 ORANGE     = { 1.0f, 0.5f, 0.0f };
    inline constexpr Vec3 PINK       = { 1.0f, 0.0f, 0.5f };
    inline constexpr Vec3 LIME       = { 0.5f, 1.0f, 0.0f };
    inline constexpr Vec3 GREENCOLD  = { 0.0f, 1.0f, 0.5f };
    inline constexpr Vec3 VIOLET     = { 0.5f, 0.0f, 1.0f };
    inline constexpr Vec3 LIGHTBLUE  = { 0.0f, 0.5f, 1.0f };
}

namespace
{
    int width = 1280;
    int height = 720;
    float zoom = 10.0f;
    float pan_y = 8.0f;

    // int width = 1280;
    // int height = 720;
    // float zoom = 2.0f;
    // float pan_y = 0.0f;

    GLFWwindow* window = NULL;

    bool pause = false;
    bool step = false;

    bool WindowUpdateLoop = true;

    int key_m1 = 0;
    int key_m2 = 0;
    bool keypressed[400] = {};
    bool keyreleased[400] = {};
}

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)window;
    (void)scancode;
    (void)action;
    (void)mods;

    if (action == GLFW_PRESS)
        keypressed[key] = true;

    if (action == GLFW_PRESS)
        keyreleased[key] = true;
}
bool GetKey(int key)
{
    return glfwGetKey(window, key) == GLFW_PRESS;
}
bool GetKeyPressed(int key)
{
    return keypressed[key] == 1;
}
bool GetKeyReleased(int key)
{
    return keyreleased[key] == 1;
}
int GetNumKeyPressed()
{
    if (GetKeyPressed(GLFW_KEY_0)) return 0;
    if (GetKeyPressed(GLFW_KEY_1)) return 1;
    if (GetKeyPressed(GLFW_KEY_2)) return 2;
    if (GetKeyPressed(GLFW_KEY_3)) return 3;
    if (GetKeyPressed(GLFW_KEY_4)) return 4;
    if (GetKeyPressed(GLFW_KEY_5)) return 5;
    if (GetKeyPressed(GLFW_KEY_6)) return 6;
    if (GetKeyPressed(GLFW_KEY_7)) return 7;
    if (GetKeyPressed(GLFW_KEY_8)) return 8;
    if (GetKeyPressed(GLFW_KEY_9)) return 9;
    return -1;
}
void ClearScreen()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void GuiStart()
{
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}
void GuiEnd()
{
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
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
Vec2 WindowGetMousePositon()
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
void DrawPoint(Vec2 p, Vec3 color)
{
    glPointSize(4.0f);
    glColor3f(color.x, color.y, color.z);
    glBegin(GL_POINTS);
    glVertex2f(p.x, p.y);
    glEnd();
    glPointSize(1.0f);
}
void DrawLine(Vec2 p0, Vec2 p1, Vec3 color)
{
    glColor3f(color.x, color.y, color.z);
    glBegin(GL_LINES);
    glVertex2f(p0.x, p0.y);
    glVertex2f(p1.x, p1.y);
    glEnd();
}

void WindowInit()
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
    glfwSetKeyCallback(window, KeyCallback);

    int gladStatus = gladLoadGL();
    if (gladStatus == 0)
    {
        fprintf(stderr, "Failed to load OpenGL.\n");
        glfwTerminate();
        PANIC;
    }

    glfwSwapInterval(1);
    glfwSetWindowSizeCallback(window, Reshape);

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
void WindowClose()
{
    glfwTerminate();
}
void WindowUpdate()
{
    if (GetKeyPressed(GLFW_KEY_P)) pause = !pause;
    if (GetKeyPressed(GLFW_KEY_RIGHT_BRACKET)) step = true;

    memset(keypressed, 0, sizeof(keypressed));
    memset(keyreleased, 0, sizeof(keyreleased));

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))  { if (key_m1 < 2) key_m1++; } else key_m1 = 0;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)) { if (key_m2 < 2) key_m2++; } else key_m2 = 0;

    WindowUpdateLoop = !pause || step;
    step = false;

    glfwPollEvents();
    glfwSwapBuffers(window);
}
bool WindowShouldClose()
{
    return glfwWindowShouldClose(window);
}

#define KEY_M0   (key_m1 > 0)
#define KEY_M1   (key_m2 > 0)
#define KEY_M0_P (key_m1 == 1)
#define KEY_M1_P (key_m2 == 1)
#define KEY_0 (GetKey(GLFW_KEY_0))
#define KEY_1 (GetKey(GLFW_KEY_1))
#define KEY_2 (GetKey(GLFW_KEY_2))
#define KEY_3 (GetKey(GLFW_KEY_3))
#define KEY_4 (GetKey(GLFW_KEY_4))
#define KEY_5 (GetKey(GLFW_KEY_5))
#define KEY_6 (GetKey(GLFW_KEY_6))
#define KEY_7 (GetKey(GLFW_KEY_7))
#define KEY_8 (GetKey(GLFW_KEY_8))
#define KEY_9 (GetKey(GLFW_KEY_9))
#define KEY_0_P (GetKeyPressed(GLFW_KEY_0))
#define KEY_1_P (GetKeyPressed(GLFW_KEY_1))
#define KEY_2_P (GetKeyPressed(GLFW_KEY_2))
#define KEY_3_P (GetKeyPressed(GLFW_KEY_3))
#define KEY_4_P (GetKeyPressed(GLFW_KEY_4))
#define KEY_5_P (GetKeyPressed(GLFW_KEY_5))
#define KEY_6_P (GetKeyPressed(GLFW_KEY_6))
#define KEY_7_P (GetKeyPressed(GLFW_KEY_7))
#define KEY_8_P (GetKeyPressed(GLFW_KEY_8))
#define KEY_9_P (GetKeyPressed(GLFW_KEY_9))
#define KEY_F_P (GetKeyPressed(GLFW_KEY_F))
#define KEY_ESC (GetKey(GLFW_KEY_ESCAPE))
