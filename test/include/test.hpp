//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP

#include <spdlog/spdlog.h>

#include "box3d.hpp"
#include "draw.hpp"


struct Settings;
class Test;
struct AnimationGenerator;

struct ContactPoint
{
    b3Fixture* fixtureA;
    b3Fixture* fixtureB;
    b3Vec3r normal;
    b3Vec3r position;
    float normalImpulse;
    float tangentImpulse;
    float separation;
};


const int32 k_max_contact_points = 2048;


class Test: public b3ContactListener {

protected:

    b3World* m_world;

    ContactPoint m_points[k_max_contact_points];
    int32 m_point_count = 0;

public:

    Test();

    virtual ~Test() = default;

    virtual void step(Settings& settings);

    virtual void pre_solve(b3Contact* contact, const b3Manifold* old_manifold);

};


using TestCreateFcn = Test*();

struct TestEntry
{
    const char* category;

    const char* name;

    TestCreateFcn* create_fcn;
};

int register_test(const char* category, const char* name, TestCreateFcn* fcn);

#define MAX_TEST 256

extern TestEntry g_test_entries[MAX_TEST];
extern int g_test_count;
extern AnimationGenerator g_animation_generator;

#endif //BOX3D_TEST_HPP
