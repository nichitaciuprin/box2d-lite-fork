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

template<typename T>
inline void Swap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

static void Flip(FeaturePair& fp)
{
    Swap(fp.e.edge1in, fp.e.edge2in);
    Swap(fp.e.edge1out, fp.e.edge2out);
}
static void ComputeIncidentEdge(ClipVertex vOut[2], Vec2 h, Vec2 pos, Mat22 rot, Vec2 normal)
{
    // the normal is from the reference box
    // convert it to the incident boxe's frame and flip sign

    normal = -(rot.Transpose() * normal);

    if (Abs(normal.x) > Abs(normal.y))
    {
        if (Sign(normal.x) > 0.0f)
        {
            vOut[0].v.Set(h.x, -h.y);
            vOut[0].fp.e.edge2in = EDGE3;
            vOut[0].fp.e.edge2out = EDGE4;

            vOut[1].v.Set(h.x, h.y);
            vOut[1].fp.e.edge2in = EDGE4;
            vOut[1].fp.e.edge2out = EDGE1;
        }
        else
        {
            vOut[0].v.Set(-h.x, h.y);
            vOut[0].fp.e.edge2in = EDGE1;
            vOut[0].fp.e.edge2out = EDGE2;

            vOut[1].v.Set(-h.x, -h.y);
            vOut[1].fp.e.edge2in = EDGE2;
            vOut[1].fp.e.edge2out = EDGE3;
        }
    }
    else
    {
        if (Sign(normal.y) > 0.0f)
        {
            vOut[0].v.Set(h.x, h.y);
            vOut[0].fp.e.edge2in = EDGE4;
            vOut[0].fp.e.edge2out = EDGE1;

            vOut[1].v.Set(-h.x, h.y);
            vOut[1].fp.e.edge2in = EDGE1;
            vOut[1].fp.e.edge2out = EDGE2;
        }
        else
        {
            vOut[0].v.Set(-h.x, -h.y);
            vOut[0].fp.e.edge2in = EDGE2;
            vOut[0].fp.e.edge2out = EDGE3;

            vOut[1].v.Set(h.x, -h.y);
            vOut[1].fp.e.edge2in = EDGE3;
            vOut[1].fp.e.edge2out = EDGE4;
        }
    }

    vOut[0].v = pos + rot * vOut[0].v;
    vOut[1].v = pos + rot * vOut[1].v;
}
static int ClipSegmentToLine(ClipVertex vIn[2], ClipVertex vOut[2], Vec2 normal, float offset, char clipEdge)
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
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

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
    Mat22 rot1t = rot1.Transpose();
    Mat22 rot2t = rot2.Transpose();
    Vec2 d1 = rot1t * (pos2 - pos1);
    Vec2 d2 = rot2t * (pos2 - pos1);
    Mat22 absC = Abs(rot1t * rot2);
    Mat22 absCT = absC.Transpose();
    Vec2 face1 = Abs(d1) - scale1 - absC  * scale2;
    Vec2 face2 = Abs(d2) - scale2 - absCT * scale1;

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

    Vec2 pos1 = body1->position;
    Vec2 pos2 = body2->position;
    Vec2 scale1 = body1->scale * 0.5f;
    Vec2 scale2 = body2->scale * 0.5f;
    Mat22 rot1 = Mat22(body1->rotation);
    Mat22 rot2 = Mat22(body2->rotation);

    Vec2 normal; float dist; Axis axis;
    auto hit = Sat(body1, body2, normal, dist, axis);
    if (!hit) return 0;

    // Setup clipping plane data based on the separating axis
    Vec2 normalFront, normalSide;
    ClipVertex incidentEdge[2] = {};
    float front;
    float sideNeg, sidePos;
    char edgeNeg, edgePos;

    // Compute the clipping lines and the line segment to be clipped.
    switch (axis)
    {
        case FACE_A_X:
        {
            normalFront = normal;
            front = Dot(pos1, normalFront) + scale1.x;
            normalSide = rot1.col2;
            float side = Dot(pos1, normalSide);
            sideNeg = -side + scale1.y;
            sidePos = +side + scale1.y;
            edgeNeg = EDGE3;
            edgePos = EDGE1;
            ComputeIncidentEdge(incidentEdge, scale2, pos2, rot2, normalFront);
        }
        break;

        case FACE_A_Y:
        {
            normalFront = normal;
            front = Dot(pos1, normalFront) + scale1.y;
            normalSide = rot1.col1;
            float side = Dot(pos1, normalSide);
            sideNeg = -side + scale1.x;
            sidePos = +side + scale1.x;
            edgeNeg = EDGE2;
            edgePos = EDGE4;
            ComputeIncidentEdge(incidentEdge, scale2, pos2, rot2, normalFront);
        }
        break;

        case FACE_B_X:
        {
            normalFront = -normal;
            front = Dot(pos2, normalFront) + scale2.x;
            normalSide = rot2.col2;
            float side = Dot(pos2, normalSide);
            sideNeg = -side + scale2.y;
            sidePos = +side + scale2.y;
            edgeNeg = EDGE3;
            edgePos = EDGE1;
            ComputeIncidentEdge(incidentEdge, scale1, pos1, rot1, normalFront);
        }
        break;

        case FACE_B_Y:
        {
            normalFront = -normal;
            front = Dot(pos2, normalFront) + scale2.y;
            normalSide = rot2.col1;
            float side = Dot(pos2, normalSide);
            sideNeg = -side + scale2.x;
            sidePos = +side + scale2.x;
            edgeNeg = EDGE2;
            edgePos = EDGE4;
            ComputeIncidentEdge(incidentEdge, scale1, pos1, rot1, normalFront);
        }
        break;
    }

    // clip other face with 5 box planes (1 face plane, 4 edge planes)

    ClipVertex clipPoints1[2] = {};
    ClipVertex clipPoints2[2] = {};
    int np;

    // Clip to box side 1
    np = ClipSegmentToLine(incidentEdge, clipPoints1, -normalSide, sideNeg, edgeNeg);
    if (np < 2) return 0;

    // Clip to negative box side 1
    np = ClipSegmentToLine(clipPoints1, clipPoints2, normalSide, sidePos, edgePos);
    if (np < 2) return 0;

    // Now clipPoints2 contains the clipping points.
    // Due to roundoff, it is possible that clipping removes all points.

    int numContacts = 0;

    for (int i = 0; i < 2; i++)
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
            Flip(contact.feature);

        numContacts++;
    }

    return numContacts;
}
