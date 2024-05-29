#include "test.hpp"
#include <random>

class TestPositionSolver: public Test {

    static constexpr int32 sphere_count = 2;
    static constexpr int32 box_count = 1;
    const real sphere_hf = 1.0f;
    const real box_hf = 1.0f;

    b3Body* m_spheres[sphere_count];
    b3Body* m_boxes[box_count];


public:

    TestPositionSolver() {

        m_world->set_gravity(b3Vec3r(0, 0, -10));


//        for (int32 i = 0; i < sphere_count; ++i) {
//            b3BodyDef body_def;
//            body_def.m_type = b3BodyType::b3_dynamic_body;
//
//            b3Vec3r p(0, 0, 1 + i * sphere_hf * 2.0f * 0.9f);
//            b3Vec3r q(0, 0, 0);
//            b3Vec3r v(0, 0, 0);
//            b3Vec3r w(0, 0, 0);
//
//            body_def.set_init_pose(p, q);
//            body_def.set_init_velocity(v, w);
//
//            b3Body* sphere = m_world->create_body(body_def);
//
//            b3SphereShape sphere_shape;
//            sphere_shape.set_as_sphere(sphere_hf);
//
//            b3FixtureDef box_fd;
//            box_fd.m_shape = &sphere_shape;
//            box_fd.m_friction = 0.3;
//            box_fd.m_restitution = 1.0;
//            box_fd.m_density = 1.0;
//
//            sphere->create_fixture(box_fd);
//            m_spheres[i] = sphere;
//        }
//
        {

            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, 0, sphere_hf);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 2, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* sphere = m_world->create_body(body_def);

            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(sphere_hf);

            b3FixtureDef box_fd;
            box_fd.m_shape = &sphere_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            sphere->create_fixture(box_fd);
        }

//        for (int32 i = 0; i < box_count; ++i) {
//            b3BodyDef body_def;
//            body_def.m_type = b3BodyType::b3_dynamic_body;
//
//            b3Vec3r p(0, 5, 1 + i * box_hf * 2.0f);
//            b3Vec3r q(0, 0, 0);
//            b3Vec3r v(0, 0, 0);
//            b3Vec3r w(0, 0, 0);
//
//            body_def.set_init_pose(p, q);
//            body_def.set_init_velocity(v, w);
//
//            b3Body* cube = m_world->create_body(body_def);
//
//            b3CubeShape cube_shape;
//            cube_shape.set_as_box(box_hf, box_hf, box_hf);
//
//            b3FixtureDef box_fd;
//            box_fd.m_shape = &cube_shape;
//            box_fd.m_friction = 0.3;
//            box_fd.m_restitution = 1.0;
//            box_fd.m_density = 1.0;
//
//            cube->create_fixture(box_fd);
//            m_boxes[i] = cube;
//        }

        ////////////////////////////////////////////////////

        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.3;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;


        ground_body->create_fixture(ground_fd);

    }

    void step(Settings &settings) override {
        Test::step(settings);
//        for (int32 i = 0; i < sphere_count; ++i) {
//            b3Body* cube = m_spheres[i];
//            b3Vec3r v = cube->get_linear_velocity();
//            b3Vec3r w = cube->get_angular_velocity();
//            // spdlog::info("cube[{}] v = ({}, {}, {}), w = ({}, {}, {})\n", i, v.x, v.y, v.z, w.x, w.y, w.z);
//        }

    }


    static Test* create() {
        return new TestPositionSolver;
    }

};

static int test_index = register_test("Functional Test", "Position Solver", TestPositionSolver::create);
