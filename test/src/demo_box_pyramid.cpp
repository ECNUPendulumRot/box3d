#include "test.hpp"
#include <random>

class DemoBoxSinglePyramid: public Test {

    const real box_hf = 1.5f;

public:

    DemoBoxSinglePyramid() {

        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;
        ground_bd.m_init_p = b3Vec3r(0, 0, -box_hf);
        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.3;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;

        ground_body->create_fixture(ground_fd);

        ////////////////////////////////////////////////////

        create_pyramid(4);

    }

    void step(Settings &settings) override {
        Test::step(settings);
    }

    void create_pyramid(int32 last_layer_count) {
        int32 layer_count = last_layer_count;
        b3Vec3r start_position = b3Vec3r(0, 0, 0);
        real box_l = 2 * box_hf;
        real delta = box_l / 3.0;
        while (layer_count != 0) {
            for (int32 i = 0; i < layer_count; ++i) {

                b3BodyDef body_def;

                body_def.m_type = b3BodyType::b3_dynamic_body;

                b3Vec3r p = start_position + b3Vec3r{0, box_l * i, 0} + b3Vec3r{0, delta * i, 0};
                b3Vec3r q(0, 0, 0);
                b3Vec3r v(0, 0, 0);
                b3Vec3r w(0, 0, 0);

                body_def.set_init_pose(p, q);
                body_def.set_init_velocity(v, w);

                b3Body* cube = m_world->create_body(body_def);

                b3CubeShape cube_shape;
                cube_shape.set_as_box(box_hf, box_hf, box_hf);

                b3FixtureDef box_fd;
                box_fd.m_shape = &cube_shape;
                box_fd.m_friction = 0.0;
                box_fd.m_restitution = 1.0;
                box_fd.m_density = 1.0;

                cube->create_fixture(box_fd);
            }
            layer_count--;
            start_position += b3Vec3r{0, 2 * delta, 0};
        }


    }


    static Test* create() {
        return new DemoBoxSinglePyramid;
    }

};

static int test_index = register_test("Demo", "Single Cube Pyramid", DemoBoxSinglePyramid::create);
