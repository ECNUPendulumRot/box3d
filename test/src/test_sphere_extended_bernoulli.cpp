#include "test.hpp"

class TestExtendedBernoulli :public Test {

    int32 sphere_count = 4;

    real radius = 0.5;

public:

    TestExtendedBernoulli() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));
        m_body_count = sphere_count + 1; // 1 dynamic body

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

        m_bodies[0] = m_world->create_body(body_def);
        m_bodies[0]->create_fixture(fixture_def);
        m_names[0] = "init_velocity_sphere";

        v = { 0, 0, 0 };

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        p = {0, -radius * sphere_count + radius, 0.5};
        for (int i = 1; i <= sphere_count; i++) {
            body_def.set_init_pose(p, q);
            m_bodies[i] = m_world->create_body(body_def);
            m_bodies[i]->create_fixture(fixture_def);

            m_names[i] = "sphere_" + std::to_string(i);

            p.y += 2 * radius;
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