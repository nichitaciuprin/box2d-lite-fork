#pragma once

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
    static constexpr Vec2 gravity = { 0, -10.0f };
    static constexpr int iterations = 10;

    vector<Body> bodie_s;
    vector<Joint> joint_s;
    map<int, Collision> collision_s;
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
        // case 0: { printf("UNREACHABLE\n"); return true; } // UNREACHABLE, clip line called after sat
        case 0: UNREACHABLE // clip line called after sat
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
            contact.separation = -separation;
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
            contact.separation = -separation;
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
        float r1l = LengthSqr(r1);
        float r2l = LengthSqr(r2);

        float massInvSum = collision.body1->massInv + collision.body2->massInv;

        float massNormal  = massInvSum + collision.body1->inertiaInv * (r1l - r1nl) + collision.body2->inertiaInv * (r2l - r2nl);
        float massTangent = massInvSum + collision.body1->inertiaInv * (r1l - r1tl) + collision.body2->inertiaInv * (r2l - r2tl);

        c->massNormalInv  = 1.0f / massNormal;
        c->massTangentInv = 1.0f / massTangent;

        if (Config::positionCorrection)
        {
            float allowedPenetration = 0.01f;
            float biasFactor = 0.2f;
            c->bias = Max(0.0f, c->separation - allowedPenetration) * biasFactor * dti;
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
            Vec2 normal = c->normal;
            auto vr = CalcRelativeVelocity(collision.body1, collision.body2, c->r1, c->r2);
            float impInit = (-Dot(normal, vr) + c->bias) * c->massNormalInv;
            float impOld = c->pn;
            float impNew = Max(0.0f, impOld + impInit);
            UpdateVelocity(collision.body1, collision.body2, c->r1, c->r2, normal * (impNew - impOld));
            c->pn = impNew;
        }

        float frictionMax = collision.friction * c->pn;

        {
            Vec2 tangent = RotateRight(c->normal);
            auto vr = CalcRelativeVelocity(collision.body1, collision.body2, c->r1, c->r2);
            float impInit = -Dot(tangent, vr) * c->massTangentInv;
            float impOld = c->pt;
            float impNew = Clamp(impOld + impInit, -frictionMax, +frictionMax);
            UpdateVelocity(collision.body1, collision.body2, c->r1, c->r2, tangent * (impNew - impOld));
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

void BodySetMass(Body* body, float mass)
{
    if (mass == FLT_MAX)
    {
        body->mass = FLT_MAX;
        body->massInv = 0.0f;
        body->inertia = FLT_MAX;
        body->inertiaInv = 0.0f;
    }
    else
    {
        body->mass = mass;
        body->massInv = 1.0f / body->mass;
        body->inertia = body->mass * LengthSqr(body->scale) / 12.0f;
        body->inertiaInv = 1.0f / body->inertia;
    }
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

        body.velocityLinear += gravity * dt;

        body.velocityLinear  += body.force  * body.massInv    * dt;
        body.velocityAngular += body.torque * body.inertiaInv * dt;

        body.force = { 0.0f, 0.0f };
        body.torque = 0.0f;
    }

    {
        for (auto& collision : collision_s) CollisionPreStep(collision.second, dti);
        for (auto& joint : joint_s) JointPreStep(&joint, dti);
    }
    for (int i = 0; i < iterations; i++)
    {
        for (auto& collision : collision_s) CollisionApplyImpulse(collision.second);
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
}
Body* CreateBox()
{
    Body body;
    bodie_s.push_back(body);
    return &bodie_s.back();
}
Body* CreateBoxStatic(Vec2 position, float rotation, Vec2 scale)
{
    auto box = CreateBox();

    box->position = position;
    box->rotation = rotation;
    box->scale = scale;

    box->velocityLinear = { 0.0f, 0.0f };
    box->velocityAngular = 0.0f;

    box->force = { 0.0f, 0.0f };
    box->torque = 0.0f;

    box->friction = 0.2f;

    box->mass = FLT_MAX;
    box->inertia = FLT_MAX;

    box->massInv = 0.0f;
    box->inertiaInv = 0.0f;

    return box;
}
Body* CreateBoxDynamic(Vec2 position, float rotation, Vec2 scale, float mass)
{
    auto box = CreateBox();

    box->position = position;
    box->rotation = rotation;
    box->scale = scale;

    box->velocityLinear = { 0.0f, 0.0f };
    box->velocityAngular = 0.0f;

    box->force = { 0.0f, 0.0f };
    box->torque = 0.0f;

    box->friction = 0.2f;

    box->mass = mass;
    box->inertia = mass * LengthSqr(scale) / 12.0f;

    box->massInv = 1.0f / mass;
    box->inertiaInv = 1.0f / box->inertia;

    return box;
}
Joint* CreateJoint(Body* b1, Body* b2, Vec2 anchor)
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

    joint_s.push_back(joint);

    return &joint_s.back();
}
Body* CreateGround()
{
    float rotation = 0.0f;
    Vec2 scale = { 100.0f, 20.0f };
    Vec2 position = { 0.0f, scale.y * -0.5f };
    return CreateBoxStatic(position, rotation, scale);
}
