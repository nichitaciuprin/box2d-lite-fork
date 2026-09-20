#include "Core.h"

struct Body
{
    Vec2 position;
    float rotation;
    Vec2 scale;
    Vec2 velocityLinear;
    float velocityAngular;
    Vec2 force;
    float torque;
    float friction;
    float mass;
    float massInv;
    float inertia;
    float inertiaInv;
};
struct Joint
{
    Mat22 m;
    Vec2 localAnchor1;
    Vec2 localAnchor2;
    Vec2 r1;
    Vec2 r2;
    Vec2 bias;
    Vec2 p;		// accumulated impulse
    Body* body1;
    Body* body2;
    float biasFactor;
    float softness;
};
struct Contact
{
    Vec2 position;
    Vec2 normal;
    Vec2 r1;
    Vec2 r2;
    float separation;
    float pn;	// accumulated normal impulse
    float pt;	// accumulated tangent impulse
    float massNormalInv;
    float massTangentInv;
    float bias;
};
struct Collision
{
    Contact contact_s[2];
    int contact_num;
    Body* body1;
    Body* body2;
    float friction; // combined friction
};

namespace
{
    float timestep = 1.0f / 60.0f;

    int demoIndex = 0;

    Body* bomb = NULL;

    int closeBodyIndex = -1;
    Vec2 closeBodyPoint;
    Vec2 closeBodyOffset;

    int selectedBodyIndex = -1;
    Vec2 selectedBodyPoint;

    vector<Body> bodie_s;
    vector<Joint> joint_s;
    map<int, Collision> collision_s;
}

void CalcJointProp(float mass, float frequencyHz, float dampingRatio, float& softness, float& biasFactor)
{
    // frequency in radians
    float omega = frequencyHz * MATH_PI * 2.0f;

    // damping coefficient
    float d = omega * dampingRatio * mass * 2.0f;

    // spring stiffness
    float k = mass * omega * omega;

    // magic formulas
    softness =           1.0f / (d + k * timestep);
    biasFactor = k * timestep / (d + k * timestep);
}

void ComputeIncidentEdge(Vec2& p0, Vec2& p1, const Body* body, Vec2 normal)
{
    Vec2 pos = body->position;
    Vec2 scaleh = body->scale * 0.5f;
    Mat22 rot = FromAngle(body->rotation);

    normal = Transpose(rot) * normal;

    if (Abs(normal.x) > Abs(normal.y))
    {
        if (normal.x >= 0.0f)
        {
            p0 = { +scaleh.x, -scaleh.y };
            p1 = { +scaleh.x, +scaleh.y };
        }
        else
        {
            p0 = { -scaleh.x, +scaleh.y };
            p1 = { -scaleh.x, -scaleh.y };
        }
    }
    else
    {
        if (normal.y >= 0.0f)
        {
            p0 = { +scaleh.x, +scaleh.y };
            p1 = { -scaleh.x, +scaleh.y };
        }
        else
        {
            p0 = { -scaleh.x, -scaleh.y };
            p1 = { +scaleh.x, -scaleh.y };
        }
    }

    p0 = pos + rot * p0;
    p1 = pos + rot * p1;
}
bool ClipLine(Vec2& p0, Vec2& p1, Vec2 normal, float offset)
{
    float dist0 = Dot(normal, p0) - offset;
    float dist1 = Dot(normal, p1) - offset;

    int state = 0;
    if (dist0 < 0.0f) state += 1;
    if (dist1 < 0.0f) state += 2;

    switch (state)
    {
        case 0: { printf("UNREACHABLE\n"); return true; } // UNREACHABLE, clip line called after sat
        case 1: { p1 = Lerp(p0, p1, dist0 / (dist0 - dist1)); return false; }
        case 2: { p0 = Lerp(p0, p1, dist0 / (dist0 - dist1)); return false; }
        case 3: return false;
    }

    UNREACHABLE
}
bool Sat(const Body* body1, const Body* body2, Vec2& normal, float& dist, int& axis)
{
    const int FACE_A_X = 0;
    const int FACE_A_Y = 1;
    const int FACE_B_X = 2;
    const int FACE_B_Y = 3;

    Vec2 pos1 = body1->position;
    Vec2 pos2 = body2->position;
    Vec2 scale1 = body1->scale * 0.5f;
    Vec2 scale2 = body2->scale * 0.5f;
    Mat22 rot1 = FromAngle(body1->rotation);
    Mat22 rot2 = FromAngle(body2->rotation);
    Mat22 rot1i = Transpose(rot1);
    Mat22 rot2i = Transpose(rot2);
    Vec2 d1 = rot1i * (pos2 - pos1);
    Vec2 d2 = rot2i * (pos2 - pos1);
    Mat22 rotc = Abs(rot1i * rot2);
    Mat22 rotci = Transpose(rotc);
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
void FindContacts(const Body* b1, const Body* b2, Contact contact_s[2], int& contact_num)
{
    Vec2 normal; float dist; int axis;
    auto hit = Sat(b1, b2, normal, dist, axis);
    if (!hit) return;

    Vec2 pos1 = b1->position;
    Vec2 pos2 = b2->position;
    Vec2 scaleh1 = b1->scale * 0.5f;
    Vec2 scaleh2 = b2->scale * 0.5f;
    Mat22 rot1 = FromAngle(b1->rotation);
    Mat22 rot2 = FromAngle(b2->rotation);

    Vec2 p0, p1;
    Vec2 normalFront, normalSide;
    float front, sideNeg, sidePos;

    const int FACE_A_X = 0;
    const int FACE_A_Y = 1;
    const int FACE_B_X = 2;
    const int FACE_B_Y = 3;

    switch (axis)
    {
        case FACE_A_X:
        {
            ComputeIncidentEdge(p0, p1, b2, -normal);
            normalFront = normal;
            normalSide = rot1.col2;
            front   = scaleh1.x + Dot(pos1, normalFront);
            sidePos = scaleh1.y + Dot(pos1, normalSide);
            sideNeg = scaleh1.y - Dot(pos1, normalSide);
        }
        break;

        case FACE_A_Y:
        {
            ComputeIncidentEdge(p0, p1, b2, -normal);
            normalFront = normal;
            normalSide = rot1.col1;
            front   = scaleh1.y + Dot(pos1, normalFront);
            sidePos = scaleh1.x + Dot(pos1, normalSide);
            sideNeg = scaleh1.x - Dot(pos1, normalSide);
        }
        break;

        case FACE_B_X:
        {
            ComputeIncidentEdge(p0, p1, b1, normal);
            normalFront = -normal;
            normalSide = rot2.col2;
            front   = scaleh2.x + Dot(pos2, normalFront);
            sidePos = scaleh2.y + Dot(pos2, normalSide);
            sideNeg = scaleh2.y - Dot(pos2, normalSide);
        }
        break;

        case FACE_B_Y:
        {
            ComputeIncidentEdge(p0, p1, b1, normal);
            normalFront = -normal;
            normalSide = rot2.col1;
            front   = scaleh2.y + Dot(pos2, normalFront);
            sidePos = scaleh2.x + Dot(pos2, normalSide);
            sideNeg = scaleh2.x - Dot(pos2, normalSide);
        }
        break;
    }

    if (ClipLine(p0, p1, +normalSide, sidePos)) { printf("UNREACHABLE\n"); return; };
    if (ClipLine(p0, p1, -normalSide, sideNeg)) { printf("UNREACHABLE\n"); return; };

    // clamps points to reference edge
    {
        auto& p = p0;
        float separation = Dot(normalFront, p) - front;
        if (separation <= 0.0f)
        {
            auto& contact = contact_s[contact_num];
            contact.position = p - normalFront * separation;
            contact.pn = 0;
            contact.pt = 0;
            contact.normal = normal;
            contact.separation = separation;
            contact.r1 = contact.position - b1->position;
            contact.r2 = contact.position - b2->position;
            contact_num++;
        }
    }
    {
        auto& p = p1;
        float separation = Dot(normalFront, p) - front;
        if (separation <= 0.0f)
        {
            auto& contact = contact_s[contact_num];
            contact.position = p - normalFront * separation;
            contact.pn = 0;
            contact.pt = 0;
            contact.normal = normal;
            contact.separation = separation;
            contact.r1 = contact.position - b1->position;
            contact.r2 = contact.position - b2->position;
            contact_num++;
        }
    }
}

Vec2 CalcRelativeVelocity(Body* b1, Body* b2, Vec2 r1, Vec2 r2)
{
    auto vel1 = b1->velocityLinear + Cross(b1->velocityAngular, r1);
    auto vel2 = b2->velocityLinear + Cross(b2->velocityAngular, r2);
    return vel2 - vel1;
}
void UpdateVelocity(Body* b1, Body* b2, Vec2 r1, Vec2 r2, Vec2 impulse)
{
    b1->velocityLinear -= impulse * b1->massInv;
    b2->velocityLinear += impulse * b2->massInv;
    b1->velocityAngular -= Cross(r1, impulse) * b1->inertiaInv;
    b2->velocityAngular += Cross(r2, impulse) * b2->inertiaInv;
}

void CollisionPreStep(Collision& collision, float dti)
{
    for (int i = 0; i < collision.contact_num; i++)
    {
        Contact* c = collision.contact_s + i;

        Vec2 normal = c->normal;
        Vec2 tangent = RotateRight(c->normal);
        Vec2 r1 = c->r1;
        Vec2 r2 = c->r2;

        float r1n = Dot(r1, normal);
        float r1t = Dot(r1, tangent);
        float r2n = Dot(r2, normal);
        float r2t = Dot(r2, tangent);
        float r1nl = r1n * r1n;
        float r1tl = r1t * r1t;
        float r2nl = r2n * r2n;
        float r2tl = r2t * r2t;
        float r1l = LengthSqrt(r1);
        float r2l = LengthSqrt(r2);

        float massInvSum = collision.body1->massInv + collision.body2->massInv;

        float massNormal  = massInvSum + collision.body1->inertiaInv * (r1l - r1nl) + collision.body2->inertiaInv * (r2l - r2nl);
        float massTangent = massInvSum + collision.body1->inertiaInv * (r1l - r1tl) + collision.body2->inertiaInv * (r2l - r2tl);

        c->massNormalInv  = 1.0f / massNormal;
        c->massTangentInv = 1.0f / massTangent;

        if (Config::positionCorrection)
        {
            float allowedPenetration = 0.01f;
            float biasFactor = 0.2f;
            c->bias = -Min(c->separation + allowedPenetration, 0.0f) * biasFactor * dti;
        }
        else
        {
            c->bias = 0.0f;
        }

        Vec2 impulse = normal * c->pn + tangent * c->pt;
        UpdateVelocity(collision.body1, collision.body2, c->r1, c->r2, impulse);
    }
}
void JointPreStep(Joint* joint, float dti)
{
    Mat22 r1 = FromAngle(joint->body1->rotation);
    Mat22 r2 = FromAngle(joint->body2->rotation);

    joint->r1 = r1 * joint->localAnchor1;
    joint->r2 = r2 * joint->localAnchor2;

    Mat22 k1;
    k1.col1.x = joint->body1->massInv + joint->body2->massInv;
    k1.col2.x = 0.0f;
    k1.col1.y = 0.0f;
    k1.col2.y = joint->body1->massInv + joint->body2->massInv;

    Mat22 k2;
    k2.col1.x =  joint->body1->inertiaInv * joint->r1.y * joint->r1.y;
    k2.col2.x = -joint->body1->inertiaInv * joint->r1.x * joint->r1.y;
    k2.col1.y = -joint->body1->inertiaInv * joint->r1.x * joint->r1.y;
    k2.col2.y =  joint->body1->inertiaInv * joint->r1.x * joint->r1.x;

    Mat22 k3;
    k3.col1.x =  joint->body2->inertiaInv * joint->r2.y * joint->r2.y;
    k3.col2.x = -joint->body2->inertiaInv * joint->r2.x * joint->r2.y;
    k3.col1.y = -joint->body2->inertiaInv * joint->r2.x * joint->r2.y;
    k3.col2.y =  joint->body2->inertiaInv * joint->r2.x * joint->r2.x;

    Mat22 k = k1 + k2 + k3;

    k.col1.x += joint->softness;
    k.col2.y += joint->softness;

    joint->m = Invert(k);

    auto p1 = joint->body1->position + joint->r1;
    auto p2 = joint->body2->position + joint->r2;

    if (Config::positionCorrection)
        joint->bias = (p2 - p1) * -joint->biasFactor * dti;
    else
        joint->bias = { 0.0f, 0.0f };

    if (Config::warmStarting)
        UpdateVelocity(joint->body1, joint->body2, joint->r1, joint->r2, joint->p);
    else
        joint->p = { 0.0f, 0.0f };
}

void CollisionApplyImpulse(Collision& collision)
{
    for (int i = 0; i < collision.contact_num; i++)
    {
        Contact* c = collision.contact_s + i;

        {
            auto vr = CalcRelativeVelocity(collision.body1, collision.body2, c->r1, c->r2);
            Vec2 normal = c->normal;
            float impInit = (-Dot(normal, vr) + c->bias) * c->massNormalInv;
            float impOld = c->pn;
            float impNew = Max(impOld + impInit, 0.0f);
            float impDiff = impNew - impOld;
            Vec2 imp = normal * impDiff;
            UpdateVelocity(collision.body1, collision.body2, c->r1, c->r2, imp);
            c->pn = impNew;
        }

        float frictionMax = collision.friction * c->pn;

        {
            auto vr = CalcRelativeVelocity(collision.body1, collision.body2, c->r1, c->r2);
            Vec2 tangent = RotateRight(c->normal);
            float impInit = -Dot(tangent, vr) * c->massTangentInv;
            float impOld = c->pt;
            float impNew = Clamp(impOld + impInit, -frictionMax, +frictionMax);
            float impDiff = impNew - impOld;
            Vec2 imp = tangent * impDiff;
            UpdateVelocity(collision.body1, collision.body2, c->r1, c->r2, imp);
            c->pt = impNew;
        }
    }
}
void JointApplyImpulse(Joint* joint)
{
    auto vr = CalcRelativeVelocity(joint->body1, joint->body2, joint->r1, joint->r2);

    auto impOld = joint->p;
    auto impNew = joint->m * (joint->bias - vr - impOld * joint->softness);

    UpdateVelocity(joint->body1, joint->body2, joint->r1, joint->r2, impNew);

    joint->p = impNew;
}

void BodyAddForce(Body* body, Vec2 force)
{
    body->force += force;
}
void BodyApplyImpulse(Body* body, Vec2 position, Vec2 velocity)
{
    auto velocityLinearNew = velocity;
    auto velocityAngularNew = Cross(position - body->position, velocity);
    body->velocityLinear += velocityLinearNew;
    body->velocityAngular += velocityAngularNew;
}

Body BodyCreate(Vec2 scale, float mass)
{
    Body body;

    body.position = { 0.0f, 0.0f };
    body.rotation = 0.0f;
    body.scale = scale;

    body.velocityLinear = { 0.0f, 0.0f };
    body.velocityAngular = 0.0f;

    body.force = { 0.0f, 0.0f };
    body.torque = 0.0f;

    body.friction = 0.2f;

    if (mass == FLT_MAX)
    {
        body.mass = FLT_MAX;
        body.massInv = 0.0f;
        body.inertia = FLT_MAX;
        body.inertiaInv = 0.0f;
    }
    else
    {
        body.mass = mass;
        body.massInv = 1.0f / body.mass;
        body.inertia = body.mass * (body.scale.x * body.scale.x + body.scale.y * body.scale.y) / 12.0f;
        body.inertiaInv = 1.0f / body.inertia;
    }

    return body;
}
Joint JointCreate(Body* b1, Body* b2, Vec2 anchor)
{
    Joint joint;

    joint.p = { 0.0f, 0.0f };

    joint.softness = 0.0f;
    joint.biasFactor = 0.2f;

    joint.body1 = b1;
    joint.body2 = b2;

    Mat22 r1 = FromAngle(b1->rotation);
    Mat22 r2 = FromAngle(b2->rotation);
    Mat22 r1i = Transpose(r1);
    Mat22 r2i = Transpose(r2);

    joint.localAnchor1 = r1i * (anchor - b1->position);
    joint.localAnchor2 = r2i * (anchor - b2->position);

    return joint;
}

Collision Collide(Body* b1, Body* b2)
{
    Collision collision;

    collision.body1 = b1;
    collision.body2 = b2;
    collision.contact_num = 0;

    FindContacts(collision.body1, collision.body2, collision.contact_s, collision.contact_num);

    // TODO contact_num checked downstream
    if (collision.contact_num == 0)
        return collision;

    collision.friction = sqrtf(b1->friction * b2->friction);

    return collision;
}

void BroadPhase()
{
    for (int i =   0; i < (int)bodie_s.size(); i++)
    for (int j = i+1; j < (int)bodie_s.size(); j++)
    {
        auto b1 = &bodie_s[i];
        auto b2 = &bodie_s[j];

        if (b1->massInv == 0.0f && b2->massInv == 0.0f) continue;

        int key = i << 16 | j;
        Collision collision = Collide(b1, b2);

        if (collision.contact_num == 0)
        {
            collision_s.erase(key);
            continue;
        }

        auto iter = collision_s.find(key);

        if (iter == collision_s.end())
        {
            collision_s.insert({ key, collision });
            continue;
        }

        auto a_old = &iter->second;
        auto a_new = &collision;

        if (Config::warmStarting)
        {
            for (int i = 0; i < a_new->contact_num; i++)
            {
                auto& c_new = a_new->contact_s[i];

                int closest = -1;
                {
                    float dist0 = 0.05f;

                    for (int j = 0; j < a_old->contact_num; j++)
                    {
                        auto& c_old = a_old->contact_s[j];

                        float dist1 = DistSqrt(c_old.position, c_new.position);

                        if (dist0 > dist1)
                        {
                            dist0 = dist1;
                            closest = j;
                        }
                    }
                }

                if (closest == -1) continue;

                c_new.pn = a_old->contact_s[closest].pn;
                c_new.pt = a_old->contact_s[closest].pt;
            }
        }

        *a_old = *a_new;
    }
}
void Step(float dt)
{
    float dti = dt > 0.0f ? 1.0f / dt : 0.0f;

    BroadPhase();

    for (auto& body : bodie_s)
    {
        if (body.massInv == 0.0f) continue;

        body.velocityLinear += Config::gravity * dt;

        body.velocityLinear  += body.force  * body.massInv    * dt;
        body.velocityAngular += body.torque * body.inertiaInv * dt;

        body.force = { 0.0f, 0.0f };
        body.torque = 0.0f;
    }

    {
        for (auto& arbiter : collision_s) CollisionPreStep(arbiter.second, dti);
        for (auto& joint : joint_s) JointPreStep(&joint, dti);
    }
    for (int i = 0; i < Config::iterations; i++)
    {
        for (auto& arbiter : collision_s) CollisionApplyImpulse(arbiter.second);
        for (auto& joint : joint_s) JointApplyImpulse(&joint);
    }

    for (auto& body : bodie_s)
    {
        if (body.massInv == 0.0f) continue;

        body.position += body.velocityLinear  * dt;
        body.rotation += body.velocityAngular * dt;
    }
}

void Clear()
{
    bodie_s.clear();
    joint_s.clear();
    collision_s.clear();
    bomb = NULL;
}
Body* CreateGround()
{
    auto body = BodyCreate({ 100.0f, 20.0f }, FLT_MAX);
    body.position = { 0.0f, body.scale.y * -0.5f };
    bodie_s.push_back(body);
    return &bodie_s.back();
}
Body* CreateBoxDynamic(Vec2 position, float rotation, Vec2 scale, float mass)
{
    auto body = BodyCreate(scale, mass);
    body.position = position;
    body.rotation = rotation;
    bodie_s.push_back(body);
    return &bodie_s.back();
}
Body* CreateBoxStatic(Vec2 position, float rotation, Vec2 scale)
{
    auto body = BodyCreate(scale, FLT_MAX);
    body.position = position;
    body.rotation = rotation;
    bodie_s.push_back(body);
    return &bodie_s.back();
}
Joint* CreateJoint(Body* b1, Body* b2, Vec2 anchor)
{
    auto joint = JointCreate(b1, b2, anchor);
    joint_s.push_back(joint);
    return &joint_s.back();
}
void LaunchBomb()
{
    if (!bomb)
    {
        auto bomb_ = BodyCreate({ 1.0f, 1.0f }, 50.0f);
        bodie_s.push_back(bomb_);
        bomb = &bodie_s.back();
    }

    bomb->position = { Random(-15.0f, 15.0f), 15.0f };
    bomb->rotation = Random(-1.5f, 1.5f);
    bomb->velocityLinear = bomb->position * -1.5f;
    bomb->velocityAngular = Random(-20.0f, 20.0f);
}

void Demo1()
{
    CreateGround();
    CreateBoxDynamic({ 0.0f, 4.0f }, 0.0f, { 1.0f, 1.0f }, 1.0f);

    // CreateBoxStatic({ 0.0f, 0.0f }, 0.0f, { 1.0f, 1.0f });
    // CreateBoxDynamic({ +0.50f, 0.0f }, -MATH_PI / 3.0f, { 0.5f, 0.5f }, 1.0f);

    // auto b0 = CreateBoxDynamic({ -0.5f, 8.0f }, 0, { 0.5f, 0.5f }, 1.0f);
    // auto b1 = CreateBoxDynamic({ +0.5f, 6.0f }, 0, { 0.5f, 0.5f }, 1.0f);
    // auto j = CreateJoint(b0, b1, (b0->position + b1->position) / 2);
    // j->softness = 1.0f;
    // j->biasFactor = 0.0f;
}
void Demo2()
{
    CreateGround();

    for (int i = 0; i < 10; i++)
        auto b1 = CreateBoxDynamic({ Random(-0.1f, 0.1f), 0.51f + 1.05f * i }, 0.0f, { 1.0f, 1.0f }, 1.0f);
}
void Demo3()
{
    CreateGround();

    Vec2 x = { -6.0f, 0.75f };

    for (int i = 0; i < 12; i++)
    {
        Vec2 y = x;

        for (int j = i; j < 12; j++)
        {
            auto b1 = CreateBoxDynamic(y, 0.0f, { 1.0f, 1.0f }, 10.0f);

            y += { 1.125f, 0.0f };
        }

        x += { 0.5625f, 2.0f };
    }
}
void Demo4()
{
    CreateGround();
    CreateBoxStatic({ -2.0f, 11.0f }, -0.25f, { 13.0f, 0.25f });
    CreateBoxStatic({ 5.25f, 9.5f },   0.00f, { 0.25f, 1.0f });
    CreateBoxStatic({ 2.0f, 7.0f },   +0.25f, { 13.0f, 0.25f });
    CreateBoxStatic({ -5.25f, 5.5f },  0.00f, { 0.25f, 1.0f });
    CreateBoxStatic({ -2.0f, 3.0f },  -0.25f, { 13.0f, 0.25f });

    float friction[5] = { 0.75f, 0.50f, 0.35f, 0.10f, 0.0f };

    for (int i = 0; i < 5; i++)
    {
        auto b1 = CreateBoxDynamic({ -7.5f + 2.0f * i, 14.0f }, 0.0f, { 0.5f, 0.5f }, 25.0f);
        b1->friction = friction[i];
    }
}
void Demo5()
{
    auto b1 = CreateGround();
    auto b2 = CreateBoxDynamic({ 9.0f, 11.0f }, 0.0f, { 1.0f, 1.0f }, 100.0f);
    CreateJoint(b1, b2, { 0.0f, 11.0f });
}
void Demo6()
{
    float mass = 10.0f;
    float frequencyHz = 4.0f;
    float dampingRatio = 0.7f;

    float softness, biasFactor;
    CalcJointProp(mass, frequencyHz, dampingRatio, softness, biasFactor);

    auto b1 = CreateGround();

    for (int i = 0; i < 15; i++)
    {
        float y = 12.0f;

        auto b2 = CreateBoxDynamic({ 0.5f + i, y }, 0.0f, { 0.75f, 0.25f }, mass);

        auto j = CreateJoint(b1, b2, { (float)i, y });
        j->softness = softness;
        j->biasFactor = biasFactor;

        b1 = b2;
    }
}
void Demo7()
{
    float mass = 50.0f;
    float frequencyHz = 2.0f;
    float dampingRatio = 0.7f;

    float softness, biasFactor;
    CalcJointProp(mass, frequencyHz, dampingRatio, softness, biasFactor);

    CreateGround();

    int numPlanks = 15;

    for (int i = 0; i < numPlanks; i++)
        CreateBoxDynamic({ -8.5f + 1.25f * i, 5.0f }, 0.0f, { 1.0f, 0.25f }, mass);

    // auto ground = &bodies.bodies[0];
    // auto p1 = &bodies.bodies[1];
    // auto p2 = &bodies.bodies.back();

    // {
    //     auto j1 = CreateJoint(ground, p2, { -9.125f + 1.25f * i, 5.0f });
    //     j1->softness = softness;
    //     j1->biasFactor = biasFactor;
    // }

    for (int i = 0; i < numPlanks; i++)
    {
        auto j1 = CreateJoint(&bodie_s[i], &bodie_s[i+1], { -9.125f + 1.25f * i, 5.0f });
        j1->softness = softness;
        j1->biasFactor = biasFactor;
    }
    {
        auto j1 = CreateJoint(&bodie_s[numPlanks], &bodie_s[0], { -9.125f + 1.25f * numPlanks, 5.0f });
        j1->softness = softness;
        j1->biasFactor = biasFactor;
    }
}
void Demo8()
{
    auto b1 = CreateGround();
    auto b2 = CreateBoxDynamic({ 0.0f, 1.0f }, 0.0f, { 12.0f, 0.25f }, 100.0f);
    CreateBoxDynamic({ -5.0f, 2.0f }, 0.0f, { 0.5f, 0.5f }, 25.0f);
    CreateBoxDynamic({ -5.5f, 2.0f }, 0.0f, { 0.5f, 0.5f }, 25.0f);
    CreateBoxDynamic({ 5.5f, 15.0f }, 0.0f, { 1.0f, 1.0f }, 100.0f);
    CreateJoint(b1, b2, { 0.0f, 1.0f });
}
void Demo9()
{
    auto b1 = CreateGround();

    CreateBoxStatic({ -1.5f, 10.0f }, 0.0f, { 12.0f, 0.5f });
    CreateBoxStatic({ 1.0f, 6.0f }, 0.3f, { 14.0f, 0.5f });

    for (int i = 0; i < 10; i++)
    {
        auto b = CreateBoxDynamic({ -6.0f + 1.0f * i, 11.125f }, 0.0f, { 0.2f, 2.0f }, 10.0f);
        b->friction = 0.1f;
    }

    auto b2 = CreateBoxStatic({ -7.0f, 4.0f }, 0.0f, { 0.5f, 3.0f });
    auto b3 = CreateBoxDynamic({ -0.9f, 1.0f }, 0.0f, { 12.0f, 0.25f }, 20.0f);
    auto b4 = CreateBoxDynamic({ -10.0f, 15.0f }, 0.0f, { 0.5f, 0.5f }, 10.0f);
    auto b5 = CreateBoxDynamic({ 6.0f, 2.5f }, 0.0f, { 2.0f, 2.0f }, 20.0f);
    auto b6 = CreateBoxDynamic({ 6.0f, 3.6f }, 0.0f, { 2.0f, 0.2f }, 10.0f);

    b5->friction = 0.1f;

    CreateJoint(b1, b3, { -2.0f, 1.0f });
    CreateJoint(b2, b4, { -7.0f, 15.0f });
    CreateJoint(b1, b5, { 6.0f, 2.6f });
    CreateJoint(b5, b6, { 7.0f, 3.5f });
}

const char* demoNames[] =
{
    "Demo 1: Single Box",
    "Demo 2: Randomized Stacking",
    "Demo 3: Pyramid Stacking",
    "Demo 4: Varying Friction Coefficients",
    "Demo 5: Simple Pendulum",
    "Demo 6: Multi-pendulum",
    "Demo 7: Suspension Bridge",
    "Demo 8: Teeter",
    "Demo 9: Dominos",
};
void (*demos[])() =
{
    Demo1,
    Demo2,
    Demo3,
    Demo4,
    Demo5,
    Demo6,
    Demo7,
    Demo8,
    Demo9
};

void InitDemo(int index)
{
    // TODO ref body by index, not pointer, and remove this reserve
    bodie_s.reserve(256);
    joint_s.reserve(256);

    Clear();
    demoIndex = index;
    demos[index]();
}

void SelectBody(Vec2 mousePos)
{
    int index = -1;

    Vec2 offset0;
    float offset0_ls = FLT_MAX;

    for (size_t i = 0; i < bodie_s.size(); i++)
    {
        auto body = &bodie_s[i];

        if (body->mass == FLT_MAX) continue;

        Vec2 offset1 = ShortPathToSurface(mousePos, body->position, body->rotation, body->scale);
        float offset1_ls = Dot(offset1, offset1);

        if (offset0_ls <= offset1_ls) continue;

        index = i;

        offset0 = offset1;
        offset0_ls = offset1_ls;
    }

    if (index == -1) PANIC

    closeBodyIndex = index;
    closeBodyOffset = offset0;
    closeBodyPoint = mousePos + offset0;
}

void OnKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS) return;

    switch (key)
    {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;

        case GLFW_KEY_P:
            pause = !pause;
            break;

        case GLFW_KEY_RIGHT_BRACKET:
            step = true;
            break;

        case GLFW_KEY_A:
            Config::accumulateImpulses = !Config::accumulateImpulses;
            break;

        case GLFW_KEY_S:
            Config::positionCorrection = !Config::positionCorrection;
            break;

        case GLFW_KEY_D:
            Config::warmStarting = !Config::warmStarting;
            break;

        case GLFW_KEY_SPACE:
            LaunchBomb();
            break;

        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            InitDemo(key - GLFW_KEY_1);
            break;
    }
}
void OnMouse(GLFWwindow* window, int button, int action, int mods)
{
    if (action != GLFW_PRESS) return;
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    auto mousePosition = GetMousePosition();

    // AddBox(coord);

    if (closeBodyIndex == -1) return;

    if (selectedBodyIndex == -1)
    {
        selectedBodyPoint = closeBodyPoint;
        selectedBodyIndex = closeBodyIndex;
    }
    else
    {
        auto body = &bodie_s[selectedBodyIndex];
        auto point = selectedBodyPoint;
        auto velocity = mousePosition - point;
        BodyApplyImpulse(body, point, velocity);
        selectedBodyIndex = -1;
    }
}

void DrawBody(Body* body, bool selected)
{
    Mat22 r = FromAngle(body->rotation);
    Vec2 p = body->position;
    Vec2 h = body->scale * 0.5f;

    Vec2 v1 = p + r * (Vec2){ -h.x, -h.y };
    Vec2 v2 = p + r * (Vec2){ +h.x, -h.y };
    Vec2 v3 = p + r * (Vec2){ +h.x, +h.y };
    Vec2 v4 = p + r * (Vec2){ -h.x, +h.y };

    if (selected)          glColor3f(1.0f, 0.0f, 0.0f);
    else if (body == bomb) glColor3f(0.4f, 0.9f, 0.4f);
    else                   glColor3f(0.8f, 0.8f, 0.9f);

    glBegin(GL_LINE_LOOP);
    glVertex2f(v1.x, v1.y);
    glVertex2f(v2.x, v2.y);
    glVertex2f(v3.x, v3.y);
    glVertex2f(v4.x, v4.y);
    glEnd();
}
void DrawJoint(Joint* joint)
{
    auto b0 = joint->body1;
    auto b1 = joint->body2;

    auto p0 = b0->position;
    auto p1 = b1->position;

    auto p2 = p0 + FromAngle(b0->rotation) * joint->localAnchor1;
    auto p3 = p1 + FromAngle(b1->rotation) * joint->localAnchor2;

    Vec3 color = { 0.50f, 0.50f, 0.75f };

    DrawLine(p0, p2, color);
    DrawLine(p1, p3, color);
}
void DrawArbiter(Collision* arbiter)
{
    glPointSize(4.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);

    for (int i = 0; i < arbiter->contact_num; i++)
    {
        Vec2 p = arbiter->contact_s[i].position;
        glVertex2f(p.x, p.y);
    }

    glEnd();
    glPointSize(1.0f);
}
void Draw()
{
    ClearScreen();

    if (selectedBodyIndex == -1)
    {
        if (closeBodyIndex != -1)
            DrawPoint(closeBodyPoint, Color3::GREEN);
    }
    else
    {
        auto mousePos = GetMousePosition();
        DrawPoint(selectedBodyPoint, Color3::RED);
        DrawLine(selectedBodyPoint, mousePos, Color3::GREEN);
    }

    for (auto& i : bodie_s)
        DrawBody(&i, false);

    for (auto& i : joint_s)
        DrawJoint(&i);

    for (auto& i : collision_s)
        DrawArbiter(&i.second);

    GuiStart();
    {
        ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));

        // ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
        // ImGui::End();

        DrawText(5, 5, demoNames[demoIndex]);
        DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

        char buffer[512];
        sprintf(buffer, "(A) Accumulation %s",        Config::accumulateImpulses ? "ON" : "OFF"); DrawText(5, 65 + 30*0, buffer);
        sprintf(buffer, "(S) Position Correction %s", Config::positionCorrection ? "ON" : "OFF"); DrawText(5, 65 + 30*1, buffer);
        sprintf(buffer, "(D) Warm Starting %s",       Config::warmStarting       ? "ON" : "OFF"); DrawText(5, 65 + 30*2, buffer);
    }
    GuiEnd();
}

int main()
{
    InitWindow();

    glfwSetMouseButtonCallback(window, OnMouse);
    glfwSetKeyCallback(window, OnKeyboard);

    InitDemo(0);

    while (!glfwWindowShouldClose(window))
    {
        auto mousePos = GetMousePosition();

        SelectBody(mousePos);

        auto update = !pause || step; step = false;
        if (update)
            Step(timestep);

        BroadPhase();

        Draw();

        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    glfwTerminate();

    return 0;
}
