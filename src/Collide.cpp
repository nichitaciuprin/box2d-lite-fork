#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Body.h"

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

template <typename T>
inline void Swap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

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
