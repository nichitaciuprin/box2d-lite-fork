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

const float k_pi = 3.14159265358979323846264f;

struct Vec2
{
    float x, y;

    Vec2() {}
    Vec2(float x, float y) : x(x), y(y) {}

    // void Set(float x_, float y_)
    // {
    //     x = x_;
    //     y = y_;
    // }

    Vec2 operator + () { return Vec2(+x, +y); }
    Vec2 operator - () { return Vec2(-x, -y); }

    void operator += (const Vec2& v)
    {
        x += v.x;
        y += v.y;
    }

    void operator -= (const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
    }

    void operator *= (float a)
    {
        x *= a;
        y *= a;
    }

    float Length() const
    {
        return sqrtf(x * x + y * y);
    }
};
struct Mat22
{
    Mat22() {}
    Mat22(float angle)
    {
        float c = cosf(angle), s = sinf(angle);
        col1.x = c; col2.x = -s;
        col1.y = s; col2.y = c;
    }

    Mat22(const Vec2& col1, const Vec2& col2) : col1(col1), col2(col2) {}

    Mat22 Transpose() const
    {
        return Mat22(Vec2(col1.x, col2.x), Vec2(col1.y, col2.y));
    }

    Mat22 Invert() const
    {
        float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
        Mat22 B;
        float det = a * d - b * c;
        assert(det != 0.0f);
        det = 1.0f / det;
        B.col1.x =  det * d;	B.col2.x = -det * b;
        B.col1.y = -det * c;	B.col2.y =  det * a;
        return B;
    }

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

inline float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}
inline float LerpInverse(float a, float b, float x)
{
    return (x - a) / (b - a);
}
inline Vec2 Lerp(Vec2 a, Vec2 b, float t)
{
    a.x = Lerp(a.x, b.x, t);
    a.y = Lerp(a.y, b.y, t);
    return a;
}


inline float Dot(const Vec2& a, const Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}
inline float Cross(const Vec2& a, const Vec2& b)
{
    return a.x * b.y - a.y * b.x;
}
inline Vec2 Cross(const Vec2& a, float s)
{
    return Vec2(s * a.y, -s * a.x);
}
inline Vec2 Cross(float s, const Vec2& a)
{
    return Vec2(-s * a.y, s * a.x);
}
inline Vec2 RotateLeft(const Vec2& a)
{
    return Vec2(-a.y, +a.x);
}
inline Vec2 RotateRight(const Vec2& a)
{
    return Vec2(+a.y, -a.x);
}
inline float LengthSqrt(const Vec2& a)
{
    return Dot(a, a);
}

inline Vec2 operator * (const Mat22& A, const Vec2& v)
{
    return Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
}
inline Vec2 operator + (const Vec2& a, const Vec2& b)
{
    return Vec2(a.x + b.x, a.y + b.y);
}
inline Vec2 operator - (const Vec2& a, const Vec2& b)
{
    return Vec2(a.x - b.x, a.y - b.y);
}
inline Vec2 operator * (float s, const Vec2& v)
{
    return Vec2(s * v.x, s * v.y);
}
inline Vec2 operator * (const Vec2& v, float s)
{
    return Vec2(v.x * s, v.y * s);
}

inline Mat22 operator + (const Mat22& A, const Mat22& B)
{
    return Mat22(A.col1 + B.col1, A.col2 + B.col2);
}
inline Mat22 operator * (const Mat22& A, const Mat22& B)
{
    return Mat22(A * B.col1, A * B.col2);
}

inline float Abs(float a)
{
    return a > 0.0f ? a : -a;
}
inline Vec2 Abs(const Vec2& a)
{
    return Vec2(fabsf(a.x), fabsf(a.y));
}
inline Mat22 Abs(const Mat22& A)
{
    return Mat22(Abs(A.col1), Abs(A.col2));
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

inline float Random()
{
    // Random number in range [-1,1]
    float r = (float)rand();
    r /= RAND_MAX;
    r = 2.0f * r - 1.0f;
    return r;
}
inline float Random(float lo, float hi)
{
    float r = (float)rand();
    r /= RAND_MAX;
    r = (hi - lo) * r + lo;
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
