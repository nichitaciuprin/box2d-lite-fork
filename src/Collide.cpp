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
enum EdgeNumbers
{
    NO_EDGE = 0,
    EDGE1,
    EDGE2,
    EDGE3,
    EDGE4
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
static int ClipSegmentToLine(ClipVertex vIn[MAX_POINTS], ClipVertex vOut[MAX_POINTS], Vec2 normal, float offset, char clipEdge)
{
    // Start with no output points
    int numOut = 0;

    // Calculate the distance of end points to the line
    float distance0 = Dot(normal, vIn[0].v) - offset;
    float distance1 = Dot(normal, vIn[1].v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
    if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0f)
    {
        // Find intersection point of edge and plane
        float interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + (vIn[1].v - vIn[0].v) * interp;

        if (distance0 > 0.0f)
        {
            vOut[numOut].fp = vIn[0].fp;
            vOut[numOut].fp.e.edge1in = clipEdge;
            vOut[numOut].fp.e.edge2in = NO_EDGE;
        }
        else
        {
            vOut[numOut].fp = vIn[1].fp;
            vOut[numOut].fp.e.edge1out = clipEdge;
            vOut[numOut].fp.e.edge2out = NO_EDGE;
        }

        numOut++;
    }

    return numOut;
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

int Collide(Contact* contacts, Body* body1, Body* body2)
{
    // The normal points from A to B

    // printf("%f\n", body1->mass);

    Vec2 pos1 = body1->position;
    Vec2 pos2 = body2->position;
    Vec2 scaleh1 = body1->scale * 0.5f;
    Vec2 scaleh2 = body2->scale * 0.5f;
    Mat22 rot1 = Mat22(body1->rotation);
    Mat22 rot2 = Mat22(body2->rotation);

    Vec2 normal; float dist; Axis axis;
    auto hit = Sat(body1, body2, normal, dist, axis);
    if (!hit) return 0;

    // Setup clipping plane data based on the separating axis
    Vec2 normalFront, normalSide;
    ClipVertex incidentEdge[MAX_POINTS] = {};
    float front, side;
    float sideNeg, sidePos;
    char edgeNeg, edgePos;

    // switch (axis)
    // {
    //     case FACE_A_X: printf("FACE_A_X\n"); break;
    //     case FACE_A_Y: printf("FACE_A_Y\n"); break;
    //     case FACE_B_X: printf("FACE_B_X\n"); break;
    //     case FACE_B_Y: printf("FACE_B_Y\n"); break;
    // }

    // Compute the clipping lines and the line segment to be clipped.
    switch (axis)
    {
        case FACE_A_X:
        {
            ComputeIncidentEdge(body2, -normal, incidentEdge[0], incidentEdge[1]);
            normalFront = normal;
            normalSide = rot1.col2;
            front = Dot(pos1, normalFront) + scaleh1.x;
            side = Dot(pos1, normalSide);
            sideNeg = -side + scaleh1.y;
            sidePos = +side + scaleh1.y;
            edgeNeg = EDGE3;
            edgePos = EDGE1;
        }
        break;

        case FACE_A_Y:
        {
            ComputeIncidentEdge(body2, -normal, incidentEdge[0], incidentEdge[1]);
            normalFront = normal;
            normalSide = rot1.col1;
            front = Dot(pos1, normalFront) + scaleh1.y;
            side = Dot(pos1, normalSide);
            sideNeg = -side + scaleh1.x;
            sidePos = +side + scaleh1.x;
            edgeNeg = EDGE2;
            edgePos = EDGE4;
        }
        break;

        case FACE_B_X:
        {
            ComputeIncidentEdge(body1, normal, incidentEdge[0], incidentEdge[1]);
            normalFront = -normal;
            normalSide = rot2.col2;
            front = Dot(pos2, normalFront) + scaleh2.x;
            side = Dot(pos2, normalSide);
            sideNeg = -side + scaleh2.y;
            sidePos = +side + scaleh2.y;
            edgeNeg = EDGE3;
            edgePos = EDGE1;
        }
        break;

        case FACE_B_Y:
        {
            ComputeIncidentEdge(body1, normal, incidentEdge[0], incidentEdge[1]);
            normalFront = -normal;
            normalSide = rot2.col1;
            front = Dot(pos2, normalFront) + scaleh2.y;
            side = Dot(pos2, normalSide);
            sideNeg = -side + scaleh2.x;
            sidePos = +side + scaleh2.x;
            edgeNeg = EDGE2;
            edgePos = EDGE4;
        }
        break;
    }

    // clip other face with 5 box planes (1 face plane, 4 edge planes)

    ClipVertex clipPoints1[MAX_POINTS] = {};
    ClipVertex clipPoints2[MAX_POINTS] = {};

    int np;
    np = ClipSegmentToLine(incidentEdge, clipPoints1, -normalSide, sideNeg, edgeNeg); if (np < MAX_POINTS) return 0;
    np = ClipSegmentToLine(clipPoints1,  clipPoints2,  normalSide, sidePos, edgePos); if (np < MAX_POINTS) return 0;

    // Now clipPoints2 contains the clipping points.
    // Due to roundoff, it is possible that clipping removes all points.

    int numContacts = 0;

    for (int i = 0; i < MAX_POINTS; i++)
    {
        float separation = Dot(normalFront, clipPoints2[i].v) - front;

        if (separation > 0) continue;

        auto& contact = contacts[numContacts];

        contact.separation = separation;
        contact.normal = normal;

        // slide contact point onto reference face (easy to cull)
        contact.position = clipPoints2[i].v - normalFront * separation;
        contact.feature = clipPoints2[i].fp;

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
