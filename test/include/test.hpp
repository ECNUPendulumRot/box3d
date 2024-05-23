//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP

#include <spdlog/spdlog.h>

#include "box3d.hpp"
#include "draw.hpp"

#include "json.hpp"
#include <fstream>


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

    b3Fixture* m_fixtures[2048];
    int32 m_fixture_count = 0;

public:

    nlohmann::json m_meta_json;

    nlohmann::json m_frame_json;

    Test();

    virtual ~Test() {
        m_world->clear();
        delete m_world;
    }

    virtual void step(Settings& settings);

    virtual void pre_solve(b3Contact* contact, const b3Manifold* old_manifold);

    inline void add_obs_fixture(b3Fixture* fixture, const char* name) {
        using json = nlohmann::json;

        json fixture_json;

        fixture_json["name"] = name;
        b3Shape* shape = fixture->get_shape();
        switch (shape->get_type()) {
        case b3ShapeType::e_sphere: {
            fixture_json["type"] = "sphere";
            fixture_json["radius"] = fixture->get_shape()->get_radius();
            break;
        }
        case b3ShapeType::e_cube: {
            b3CubeShape* cube = (b3CubeShape*)shape;
            fixture_json["type"] = "cube";
            std::array<float, 3> hf = {cube->m_h_xyz.x, cube->m_h_xyz.y, cube->m_h_xyz.z};
            fixture_json["half_length"] = hf;
            break;
        }
        case b3ShapeType::e_plane: {
            b3PlaneShape* plane = (b3PlaneShape*)shape;
            fixture_json["type"] = "plane";
            std::array<float, 2> lw = {plane->m_half_length, plane->m_half_width};
            fixture_json["half_length"] = lw;
            break;
        }
        default:
            break;
        }

        m_meta_json.push_back(fixture_json);

        m_fixtures[m_fixture_count++] = fixture;
    }

    void clear_obs_fixture() {
        m_fixture_count = 0;
    }

    void record_frame() {

        using json = nlohmann::json;

        json frame_json;

        for (int i = 0; i < m_fixture_count; ++i) {
            b3Quaternionr q = m_fixtures[i]->get_body()->get_quaternion();
            b3Vec3r p = m_fixtures[i]->get_body()->get_position();
            json status_json;
            status_json["position"] = {p.x, p.y, p.z};
            status_json["quaternion"] = {q.m_w, q.m_x, q.m_y, q.m_z};
            frame_json[m_meta_json[i]["name"]] = status_json;
        }

        m_frame_json.push_back(frame_json);
    }
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
