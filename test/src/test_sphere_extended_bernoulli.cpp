#include "test.hpp"

class TestExtendedBernoulli :public Test {

    int32 half_sphere_count = 2;

    real radius = 0.5;

    b3Body* bodies[2048];

public:

    TestExtendedBernoulli() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // create a dynamic body
        b3Transformr pose, velocity;

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(radius);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;

        b3Vec3r p(2, 0, 0.5);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(0, 0, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);

        v = { 0, 0, 0 };

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        for (int i = 0; i < half_sphere_count; i++) {
            p = {0, radius + i * 2 * radius, 0.5};
            body_def.set_init_pose(p, q);
            m_world->create_body(body_def)->create_fixture(fixture_def);

            p = {0, -radius - i * 2 * radius, 0.5};
            body_def.set_init_pose(p, q);
            m_world->create_body(body_def)->create_fixture(fixture_def);
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

    static Test* create() {
        return new TestExtendedBernoulli;
    }

};

static int test_index = register_test("Sphere Scene Test", "Extended Bernoulli", TestExtendedBernoulli::create);