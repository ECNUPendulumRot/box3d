#include "test.hpp"
#include <random>

class TestBoxCollide: public Test {

    static constexpr int32 box_count = 2;
    const real box_hf_size = 1.0f;

    b3Body* m_bodys[box_count];

public:

    TestBoxCollide() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0,  -4.0f, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 2, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* cube = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(box_hf_size, box_hf_size, box_hf_size);

            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.0;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            cube->create_fixture(box_fd);
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0,  4.0f, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, -2, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* cube = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(box_hf_size, box_hf_size, box_hf_size);

            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.0;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            cube->create_fixture(box_fd);
        }

    }



    void step(Settings &settings) override {
        Test::step(settings);
    }


    static Test* create() {
        return new TestBoxCollide;
    }

};

static int test_index = register_test("Cube Scene Test", "Cube Collide", TestBoxCollide::create);
