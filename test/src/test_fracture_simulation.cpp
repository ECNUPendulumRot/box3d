#include "test.hpp"

class TestFractureSimulation :public Test {

public:

    TestFractureSimulation() {

        m_world->set_gravity(b3Vec3r(0, 0, -10));
        int num_of_spheres = 3;
        // create a dynamic body
        b3Transr pose, velocity;

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


        b3Vec3r p(-2, -2, 0.5);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(2, 2, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);

        p = { -2, -4, 0.5 };
        v = { -2, 2, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);
        real x = 0;



        for (int32 i = 0; i < num_of_spheres - 2; i++) {
            p = { 0, x + i, 0.5 };
            v = { 0, 0, 0 };
            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);
            m_world->create_body(body_def)->create_fixture(fixture_def);
        }
        p = { 0.6 , x + num_of_spheres - 3 + 0.7f, 0.5 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        m_world->create_body(body_def)->create_fixture(fixture_def);

        p = { -0.6 , x + num_of_spheres - 3 + 0.7f, 0.5 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        m_world->create_body(body_def)->create_fixture(fixture_def);
        // create a ground
        p = { 0, 0, 0 };
        v = { 0, 0, 0 };
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
        return new TestFractureSimulation;
    }

};

static int test_index = register_test("Sphere Scene Test", "Fracture simulation", TestFractureSimulation::create);