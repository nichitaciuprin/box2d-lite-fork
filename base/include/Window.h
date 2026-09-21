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
void UpdateWindow()
{
    glfwPollEvents();
    glfwSwapBuffers(window);
}