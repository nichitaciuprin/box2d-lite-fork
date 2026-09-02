#pragma once

#include <math.h>
#include <float.h>
#include <assert.h>
#include <stdlib.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <memory.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define PANIC { fprintf(stderr, "\033[91mPANIC %s:%d \n\033[0m" , __FILENAME__, __LINE__); _Exit(-1); }

static constexpr int MAX_POINTS = 2;

#define MATH_PI 3.14159265358979323846f

struct Vec2
{
    float x, y;
};
struct Mat22
{
    Vec2 col1, col2;
};

struct OrientedRectangle
{
    Vec2 position;
    Vec2 scale;
    float rotation;
};

template <typename T>
inline void Swap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

inline void PrintVec2(Vec2 v)
{
    printf("{ %f, %f }\n", v.x, v.y);
}

inline float Abs(float a)
{
    return a > 0.0f ? a : -a;
}
inline float Sign(float x)
{
    return x < 0.0f ? -1.0f : 1.0f;
}
inline float Min(float a, float b)
{
    return a < b ? a : b;
}
inline float Max(float a, float b)
{
    return a > b ? a : b;
}
inline float Clamp(float a, float low, float high)
{
    return Max(low, Min(a, high));
}
inline float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}
inline float LerpInverse(float a, float b, float x)
{
    return (x - a) / (b - a);
}

inline Vec2 Abs(Vec2 a)
{
    return { fabsf(a.x), fabsf(a.y) };
}
inline float Dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline float Cross(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}
inline Vec2 Cross(Vec2 a, float s)
{
    return { s * a.y, -s * a.x };
}
inline Vec2 Cross(float s, Vec2 a)
{
    return { -s * a.y, s * a.x };
}
inline Vec2 RotateLeft(Vec2 a)
{
    return { -a.y, +a.x };
}
inline Vec2 RotateRight(Vec2 a)
{
    return { +a.y, -a.x };
}
inline Vec2 Lerp(Vec2 a, Vec2 b, float t)
{
    a.x = Lerp(a.x, b.x, t);
    a.y = Lerp(a.y, b.y, t);
    return a;
}
inline float Length(Vec2 a)
{
    return sqrtf(Dot(a, a));
}
inline float LengthSqrt(Vec2 a)
{
    return Dot(a, a);
}

inline Mat22 Abs(Mat22 A)
{
    return { Abs(A.col1), Abs(A.col2) };
}
inline Mat22 Transpose(Mat22 m)
{
    return { { m.col1.x, m.col2.x }, { m.col1.y, m.col2.y } };
}
inline Mat22 Invert(Mat22 m)
{
    float a = m.col1.x;
    float b = m.col2.x;
    float c = m.col1.y;
    float d = m.col2.y;

    float det = a * d - b * c;
    assert(det != 0.0f);
    float deti = 1.0f / det;

    m.col1.x =  deti * d;
    m.col2.x = -deti * b;
    m.col1.y = -deti * c;
    m.col2.y =  deti * a;

    return m;
}
inline Mat22 FromAngle(float rad)
{
    float cos = cosf(rad);
    float sin = sinf(rad);
    return { { cos, sin }, { -sin, cos } };
}

inline Vec2 operator + (Vec2 r) { return { +r.x, +r.y }; }
inline Vec2 operator - (Vec2 r) { return { -r.x, -r.y }; }

inline Vec2 operator + (Vec2 l, Vec2 r) { return { l.x + r.x, l.y + r.y }; }
inline Vec2 operator - (Vec2 l, Vec2 r) { return { l.x - r.x, l.y - r.y }; }

inline Vec2 operator * (Vec2 l, float r) { return { l.x * r, l.y * r }; }
inline Vec2 operator * (float l, Vec2 r) { return { l * r.x, l * r.y }; }

inline Vec2 operator * (Mat22 l, Vec2 r) { return { l.col1.x * r.x + l.col2.x * r.y, l.col1.y * r.x + l.col2.y * r.y }; }

inline Mat22 operator + (Mat22 l, Mat22 r) { return { l.col1 + r.col1, l.col2 + r.col2 }; }
inline Mat22 operator * (Mat22 l, Mat22 r) { return { l * r.col1, l * r.col2 }; }

inline void operator += (Vec2& l, Vec2 r) { l.x += r.x; l.y += r.y; };
inline void operator -= (Vec2& l, Vec2 r) { l.x -= r.x; l.y -= r.y; };

inline float Random()
{
    // Random number in range [-1,1]
    float r = (float)rand();
    r /= RAND_MAX;
    r = 2.0f * r - 1.0f;
    return r;
}
inline float Random(float min, float max)
{
    float r = (float)rand();
    r /= RAND_MAX;
    r = (max - min) * r + min;
    return r;
}

inline bool IsPointInsideBox(Vec2 point, Vec2 boxPosition, float boxRotation, Vec2 boxScale)
{
    point -= boxPosition;

    float sin = sinf(-boxRotation);
    float cos = cosf(-boxRotation);

    float x = point.x * +cos + point.y * +sin;
    float y = point.x * -sin + point.y * +cos;

    point.x = x;
    point.y = y;

    float w = boxScale.x * 0.5f;
    float h = boxScale.y * 0.5f;

    if (fabsf(point.x) > w) return false;
    if (fabsf(point.y) > h) return false;

    return true;
}
inline Vec2 ShortPathToSurface(Vec2 point, Vec2 boxPosition, float boxRotation, Vec2 boxScale)
{
    Vec2 result = {};

    point -= boxPosition;

    // float sin = sinf(-boxRotation);
    // float cos = cosf(-boxRotation);

    // TODO opengl assumed
    float sin = sinf(boxRotation);
    float cos = cosf(boxRotation);

    auto pointOld = point;

    point.x = pointOld.x * +cos + pointOld.y * +sin;
    point.y = pointOld.x * -sin + pointOld.y * +cos;

    float w = boxScale.x * 0.5f;
    float h = boxScale.y * 0.5f;

    float xclose = point.x < 0.0f ? -w : +w;
    float yclose = point.y < 0.0f ? -h : +h;

    float xoffset = xclose - point.x;
    float yoffset = yclose - point.y;

    bool inside_x = fabsf(point.x) <= w;
    bool inside_y = fabsf(point.y) <= h;

    int state = 0;
    if (inside_y) state += 1;
    if (inside_x) state += 2;

    switch (state)
    {
        case 0: result = { xoffset, yoffset }; break;
        case 1: result = { xoffset, 0.0f };    break;
        case 2: result = { 0.0f, yoffset };    break;
        case 3:
        {
            if (fabsf(xoffset) < fabsf(yoffset))
                result = { xoffset, 0.0f };
            else
                result = { 0.0f, yoffset };

            break;
        }
    }

    {
        float x_ = result.x * +cos + result.y * -sin;
        float y_ = result.x * +sin + result.y * +cos;
        result.x = x_;
        result.y = y_;
    }

    return result;
}

/*
bool OverlapOnAxis(const Rectangle2D& rect1, const Rectangle2D& rect2, const vec2& axis)
{
    Interval2D a = GetInterval(rect1, axis);
    Interval2D b = GetInterval(rect2, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}
Interval GetInterval(const AABB& aabb, const vec3& axis)
{
    vec3 i = GetMin(aabb);
    vec3 a = GetMax(aabb);
    vec3 vertex[8] =
    {
        vec3(i.x, a.y, a.z),
        vec3(i.x, a.y, i.z),
        vec3(i.x, i.y, a.z),
        vec3(i.x, i.y, i.z),
        vec3(a.x, a.y, a.z),
        vec3(a.x, a.y, i.z),
        vec3(a.x, i.y, a.z),
        vec3(a.x, i.y, i.z)
    };
    result.min = result.max = Dot(axis, vertex[0]);
    for (int i = 1; i < 8; i++)
    {
        float projection = Dot(axis, vertex[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
}
Interval GetInterval(const OBB& obb, const vec3& axis)
{
    vec3 vertex[8];
    vec3 C = obb.position; // OBB Center
    vec3 E = obb.size; // OBB Extents
    const float* o = obb.orientation.asArray;
    vec3 A[] =
    {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8]),
    };
    vertex[0] = C + A[0]*E[0] + A[1]*E[1] + A[2]*E[2];
    vertex[1] = C - A[0]*E[0] + A[1]*E[1] + A[2]*E[2];
    vertex[2] = C + A[0]*E[0] - A[1]*E[1] + A[2]*E[2];
    vertex[3] = C + A[0]*E[0] + A[1]*E[1] - A[2]*E[2];
    vertex[4] = C - A[0]*E[0] - A[1]*E[1] - A[2]*E[2];
    vertex[5] = C + A[0]*E[0] - A[1]*E[1] - A[2]*E[2];
    vertex[6] = C - A[0]*E[0] + A[1]*E[1] - A[2]*E[2];
    vertex[7] = C - A[0]*E[0] - A[1]*E[1] + A[2]*E[2];
    result.min = result.max = Dot(axis, vertex[0]);
    for (int i = 1; i < 8; i++)
    {
        float projection = Dot(axis, vertex[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
}
bool RectangleOrientedRectangle(const Rectangle2D& rect1, const OrientedRectangle& rect2)
{
    vec2 axisToTest[]{
    vec2(1, 0),vec2(0, 1),
    vec2(),vec2()
    };
    float t = DEG2RAD(rect2.rotation);
    float zRot[] =
    {
         cosf(t), sinf(t),
        -sinf(t), cosf(t)
    };

    vec2 axis = Normalized(vec2(rect2.halfExtents.x, 0));
    Multiply(axisToTest[2].asArray, axis.asArray, 1, 2, zRot, 2, 2);
    axis = Normalized(vec2(0, rect2.halfExtents.y));
    Multiply(axisToTest[3].asArray, axis.asArray, 1, 2, zRot, 2, 2);

    for (int i = 0; i < 4; ++i)
    {
        if (!OverlapOnAxis(rect1, rect2, axisToTest[i]))
            return false;
    }

    return true; // We have a collision
}
bool OrientedRectangleOrientedRectangle(const OrientedRectangle& r1, const OrientedRectangle& r2)
{
    Rectangle2D local1(Point2D(), r1.halfExtents * 2.0f);
    vec2 r = r2.position - r1.position;
    OrientedRectanglelocal2(r2.position, r2.halfExtents, r2.rotation);
    local2.rotation = r2.rotation - r1.rotation;
    float t = -DEG2RAD(r1.rotation);
    float z[] =
    {
         cosf(t), +sinf(t),
        -sinf(t),  cosf(t)
    };
    Multiply(r.asArray, vec2(r.x, r.y).asArray, 1,2, z,2,2);
    local2.position = r + r1.halfExtents;
    return RectangleOrientedRectangle(local1, local2);
}
*/
