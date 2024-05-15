#include "test.hpp"

class TestBilliards :public Test {
public:
    TestBilliards() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // create a dynamic body
        b3Transformr pose, velocity;

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(0.5);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;


        b3Vec3r p(0, 0, 0.5);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(0, 5, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);

        real x = 4;
        int32 layer = 2;

        v = { 0, 0, 0 };

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        for (int i = 1; i <= layer; i++) {
            x++;
            if (i % 2 == 0) {
                for (int j = 1; j < i; j *= 2) {
                    p = { j * 0.5, x, 0.5 };
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);
                    m_world->create_body(body_def)->create_fixture(fixture_def);
                    p = { -j * 0.5, x, 0.5 };
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);
                    m_world->create_body(body_def)->create_fixture(fixture_def);
                }
            } else {
                p = { 0, x, 0.5 };
                body_def.set_init_pose(p, q);
                body_def.set_init_velocity(v, w);
                m_world->create_body(body_def)->create_fixture(fixture_def);
                for (int j = 1; j < i - 1; j *= 2) {
                    p = { j * 1.0, x, 0.5 };
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);
                    m_world->create_body(body_def)->create_fixture(fixture_def);
                    p = { -j * 1.0, x, 0.5 };
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);
                    m_world->create_body(body_def)->create_fixture(fixture_def);
                }
            }
        }

        // create a ground
        p = { 0, 0, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(50, 50);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        ground_body->create_fixture(fixture_def);

    }

    /*void create shpere(b3Vec3r p,) {

    }*/

    static Test* create() {
        return new TestBilliards;
    }

    

};

static int test_index = register_test("Sphere Scene Test", "Billiards", TestBilliards::create);