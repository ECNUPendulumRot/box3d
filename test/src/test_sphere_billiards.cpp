#include "test.hpp"

class TestBilliards :public Test {

    int layer = 4;

public:

    TestBilliards() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

        b3FixtureDef fixture_def;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1);
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, -10, 1);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 5, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* b = m_world->create_body(body_def);

            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1.0;

            b3Fixture* f = b->create_fixture(fixture_def);

            utils.track_body(b, "init_velocity_sphere");
            utils.track_fixture(f, "init_velocity_sphere");
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            int index = 1;

            real y_distance = sqrtf(3.0);
            b3Vec3r x_offset(-2, 0, 0);

            b3Vec3r q(0, 0, 0);
            for(int i = 0; i < layer; i++) {
                b3Vec3r left_position(i, i * y_distance, 1);
                for(int j = 0; j <= i; j++) {
                    b3Vec3r p = left_position + j * x_offset;
                    body_def.set_init_pose(p, q);

                    b3Body* b = m_world->create_body(body_def);
                    b3Fixture* f = b->create_fixture(fixture_def);

                    utils.track_body(b, ("sphere_" + std::to_string(index)).c_str());
                    utils.track_fixture(f, ("sphere_" + std::to_string(index)).c_str());
                    index++;
                }
            }
        }

        {
            // create a ground
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;
            b3Fixture* fg = ground_body->create_fixture(fixture_def);

            utils.track_body(ground_body, "ground");
            utils.track_fixture(fg, "ground");

        }
    }

    static Test* create() {
        return new TestBilliards;
    }
};

static int test_index = register_test("Sphere Scene Test", "Billiards", TestBilliards::create);