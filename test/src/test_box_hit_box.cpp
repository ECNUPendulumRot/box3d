#include "test.hpp"
#include <random>


class TestBoxhitbox: public Test {

    int32 box_count = 6;
    const real box_hf_size = 0.5f;

public:

    TestBoxhitbox() {


        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3Vec3r p(0, 0, 0);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(0, 0, 0);
        b3Vec3r w(0, 0, 0);

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        p = {20, 0, 1.6};
        q = {0, 0, 0};
        v = {-10, 0, 0};
        w = {0, 0, 0};
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1.6);
        b3FixtureDef sphere_fd;
        sphere_fd.m_shape = &sphere_shape;
        sphere_fd.m_friction = 0.0;
        sphere_fd.m_restitution = 1;
        sphere_fd.m_density = 1.0;

        b3Body *sphere = m_world->create_body(body_def);
        b3Fixture* fs = sphere->create_fixture(sphere_fd);
        utils.track_fixture(fs, "hitting_sphere");

        int32 scale = 1;
        real x_pos = 0;
        x_pos += box_hf_size;
        int32 index = 0;
        for (int32 k = 0; k < 3; k++) {
            for (int32 i = 0; i < 1; i++) {
                for (int32 j = 0; j < box_count / 2; j++) {

                    b3BodyDef body_def;
                    body_def.m_type = b3BodyType::b3_dynamic_body;

                    p = {x_pos, (scale * box_hf_size + j * scale * box_hf_size * 2.0f), box_hf_size * scale + i * box_hf_size * scale * 2.0f};
                    q = {0, 0, 0};
                    v = {0, 0, 0};
                    w = {0, 0, 0};
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);

                    b3CubeShape cube_shape;
                    cube_shape.set_as_box(box_hf_size, scale * box_hf_size, scale * box_hf_size);
                    b3FixtureDef box_fd;
                    box_fd.m_shape = &cube_shape;
                    box_fd.m_friction = 0.0;
                    box_fd.m_restitution = 1.0;
                    box_fd.m_density = 1.0;

                    b3Body *cube_r = m_world->create_body(body_def);
                    b3Fixture* fr = cube_r->create_fixture(box_fd);
                    utils.track_fixture(fr, ("cube" + std::to_string(index++)).c_str());

                    p = {x_pos, -(scale * box_hf_size + j * scale * box_hf_size * 2.0f), box_hf_size * scale + i * box_hf_size * scale * 2.0f};
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);
                    b3Body *cube_l = m_world->create_body(body_def);
                    b3Fixture* fl = cube_l->create_fixture(box_fd);
                    utils.track_fixture(fl, ("cube" + std::to_string(index++)).c_str());
                }
            }
            scale *= 2;
            box_count /= 2;
            x_pos += 2 * box_hf_size;
        }


        ////////////////////////////////////////////////////

        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(100, 100);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.0;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;

        b3Fixture* fg = ground_body->create_fixture(ground_fd);
        utils.track_fixture(fg, "ground");
    }


    static Test* create() {
        return new TestBoxhitbox;
    }

};

static int test_index = register_test("Cube Scene Test", "Box hit box", TestBoxhitbox::create);

