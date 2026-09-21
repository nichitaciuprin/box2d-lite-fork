#pragma once

#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl2.h"

#include <stdio.h>
#include <iostream>

#include <vector>
#include <map>

using namespace std;

template <typename T>
inline void Swap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define PANIC { fprintf(stderr, "\033[91mPANIC %s:%d \n\033[0m" , __FILENAME__, __LINE__); _Exit(-1); }

#define MATH_PI 3.14159265358979323846f

#if defined(__GNUC__) || defined(__clang__)
    #define UNREACHABLE __builtin_unreachable();
#elif defined(_MSC_VER)
    #define UNREACHABLE __assume(0);
#else
    #define UNREACHABLE ((void)0);
#endif

#include "MathUtils.h"
#include "Config.h"
#include "Window.h"
#include "Physics.h"
