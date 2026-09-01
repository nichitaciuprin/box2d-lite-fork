#pragma once

typedef pair<ArbiterKey, Arbiter> ArbPair;

struct Impulse
{
    Vec2 position;
    Vec2 velocity;
};

struct World
{
public:
    Vec2 gravity;
    int iterations;

    std::vector<Body*> bodies;
    std::vector<Joint*> joints;
    std::map<ArbiterKey, Arbiter> arbiters;

    World(Vec2 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

    void Clear()
    {
        bodies.clear();
        joints.clear();
        arbiters.clear();
    }
    void Add(Body* body)
    {
        bodies.push_back(body);
    }
    void Add(Joint* joint)
    {
        joints.push_back(joint);
    }
    void ApplyImpulse(Body* body, Vec2 position, Vec2 velocity)
    {
        auto velocityLinearNew = velocity;
        auto velocityAngularNew = Cross(position - body->position, velocity);
        // body->velocityLinear += velocityLinearNew * body->massInv;
        // body->velocityAngular += velocityAngularNew * body->inertiaInv;
        body->velocityLinear += velocityLinearNew;
        body->velocityAngular += velocityAngularNew;
    }

    void Step(float dt)
    {
        float dti = dt > 0.0f ? 1.0f / dt : 0.0f;

        BroadPhase();

        for (auto& body : bodies)
        {
            if (body->massInv == 0.0f) continue;

            body->velocityLinear += gravity * dt;

            body->velocityLinear  += body->force  * body->massInv    * dt;
            body->velocityAngular += body->torque * body->inertiaInv * dt;

            body->force = { 0.0f, 0.0f };
            body->torque = 0.0f;
        }

        {
            for (auto& arbiter : arbiters) arbiter.second.PreStep(dti);
            for (auto& joint : joints) joint->PreStep(dti);
        }
        for (int i = 0; i < iterations; i++)
        {
            for (auto& arbiter : arbiters) ArbiterApplyImpulse(arbiter.second);
            for (auto& joint : joints) joint->ApplyImpulse();
        }

        for (auto& body : bodies)
        {
            if (body->massInv == 0.0f) continue;

            body->position += body->velocityLinear  * dt;
            body->rotation += body->velocityAngular * dt;
        }
    }

    void BroadPhase()
    {
        // O(n^2) broad-phase

        for (int i =   0; i < (int)bodies.size(); i++)
        for (int j = i+1; j < (int)bodies.size(); j++)
        {
            Body* b1 = bodies[i];
            Body* b2 = bodies[j];

            if (b1->massInv == 0.0f && b2->massInv == 0.0f) continue;

            Arbiter newArb = ArbiterCreate(b1, b2);
            ArbiterKey key(b1, b2);

            if (newArb.numContacts == 0)
            {
                arbiters.erase(key);
                continue;
            }

            auto iter = arbiters.find(key);

            if (iter == arbiters.end())
            {
                arbiters.insert(ArbPair(key, newArb));
                continue;
            }

            auto a_old = &iter->second;
            auto a_new = &newArb;

            if (Config::warmStarting)
            {
                for (int i = 0; i < a_new->numContacts; i++)
                for (int j = 0; j < a_old->numContacts; j++)
                {
                    auto& c_new = a_new->contacts[i];
                    auto& c_old = a_old->contacts[j];

                    if (c_new.feature.value != c_old.feature.value) continue;

                    c_new.Pn = c_old.Pn;
                    c_new.Pt = c_old.Pt;

                    break;
                }
            }

            *a_old = *a_new;
        }
    }
};
