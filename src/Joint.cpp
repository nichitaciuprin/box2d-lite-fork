#include "box2d-lite/MathUtils.h"
#include "box2d-lite/Config.h"
#include "box2d-lite/Body.h"

#include "box2d-lite/Joint.h"

void Joint::Set(Body* b1, Body* b2, const Vec2& anchor)
{
    body1 = b1;
    body2 = b2;

    Mat22 Rot1(body1->rotation);
    Mat22 Rot2(body2->rotation);
    Mat22 Rot1T = Rot1.Transpose();
    Mat22 Rot2T = Rot2.Transpose();

    localAnchor1 = Rot1T * (anchor - body1->position);
    localAnchor2 = Rot2T * (anchor - body2->position);

    P.Set(0.0f, 0.0f);

    softness = 0.0f;
    biasFactor = 0.2f;
}

