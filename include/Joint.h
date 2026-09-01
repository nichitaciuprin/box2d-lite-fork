#pragma once

struct Joint
{
    Mat22 M;
    Vec2 localAnchor1;
    Vec2 localAnchor2;
    Vec2 r1;
    Vec2 r2;
    Vec2 bias;
    Vec2 P;		// accumulated impulse
    Body* body1;
    Body* body2;
    float biasFactor;
    float softness;

    Joint() : body1(0), body2(0), P(0.0f, 0.0f), biasFactor(0.2f), softness(0.0f) {}

    void Set(Body* b1, Body* b2, Vec2 anchor)
    {
        body1 = b1;
        body2 = b2;

        Mat22 Rot1 = Mat22(body1->rotation);
        Mat22 Rot2 = Mat22(body2->rotation);
        Mat22 Rot1T = Rot1.Transpose();
        Mat22 Rot2T = Rot2.Transpose();

        localAnchor1 = Rot1T * (anchor - body1->position);
        localAnchor2 = Rot2T * (anchor - body2->position);

        P = { 0.0f, 0.0f };

        softness = 0.0f;
        biasFactor = 0.2f;
    }
};

void JointPreStep(Joint* joint, float dti)
{
    // Pre-compute anchors, mass matrix, and bias.
    Mat22 Rot1 = Mat22(joint->body1->rotation);
    Mat22 Rot2 = Mat22(joint->body2->rotation);

    joint->r1 = Rot1 * joint->localAnchor1;
    joint->r2 = Rot2 * joint->localAnchor2;

    // deltaV = deltaV0 + K * impulse
    // invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]

    Mat22 K1;
    K1.col1.x = joint->body1->massInv + joint->body2->massInv;
    K1.col2.x = 0.0f;
    K1.col1.y = 0.0f;
    K1.col2.y = joint->body1->massInv + joint->body2->massInv;

    Mat22 K2;
    K2.col1.x =  joint->body1->inertiaInv * joint->r1.y * joint->r1.y;
    K2.col2.x = -joint->body1->inertiaInv * joint->r1.x * joint->r1.y;
    K2.col1.y = -joint->body1->inertiaInv * joint->r1.x * joint->r1.y;
    K2.col2.y =  joint->body1->inertiaInv * joint->r1.x * joint->r1.x;

    Mat22 K3;
    K3.col1.x =  joint->body2->inertiaInv * joint->r2.y * joint->r2.y;
    K3.col2.x = -joint->body2->inertiaInv * joint->r2.x * joint->r2.y;
    K3.col1.y = -joint->body2->inertiaInv * joint->r2.x * joint->r2.y;
    K3.col2.y =  joint->body2->inertiaInv * joint->r2.x * joint->r2.x;

    Mat22 K = K1 + K2 + K3;
    K.col1.x += joint->softness;
    K.col2.y += joint->softness;

    joint->M = K.Invert();

    Vec2 p1 = joint->body1->position + joint->r1;
    Vec2 p2 = joint->body2->position + joint->r2;
    Vec2 dp = p2 - p1;

    if (Config::positionCorrection)
        joint->bias = -joint->biasFactor * dti * dp;
    else
        joint->bias = { 0.0f, 0.0f };

    if (Config::warmStarting)
    {
        joint->body1->velocityLinear -= joint->P * joint->body1->massInv;
        joint->body2->velocityLinear += joint->P * joint->body2->massInv;
        joint->body1->velocityAngular -= Cross(joint->r1, joint->P) * joint->body1->inertiaInv;
        joint->body2->velocityAngular += Cross(joint->r2, joint->P) * joint->body2->inertiaInv;
    }
    else
    {
        joint->P = { 0.0f, 0.0f };
    }
}
void JointApplyImpulse(Joint* joint)
{
    Vec2 dv = joint->body2->velocityLinear + Cross(joint->body2->velocityAngular, joint->r2) - joint->body1->velocityLinear - Cross(joint->body1->velocityAngular, joint->r1);

    Vec2 impulse = joint->M * (joint->bias - dv - joint->softness * joint->P);

    joint->body1->velocityLinear -= joint->body1->massInv * impulse;
    joint->body1->velocityAngular -= joint->body1->inertiaInv * Cross(joint->r1, impulse);

    joint->body2->velocityLinear += joint->body2->massInv * impulse;
    joint->body2->velocityAngular += joint->body2->inertiaInv * Cross(joint->r2, impulse);

    joint->P += impulse;
}