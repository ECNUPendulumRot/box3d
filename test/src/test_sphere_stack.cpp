#include "test.hpp"

class TestSphereStack :public Test {

    int num = 10;

public:
    TestSphereStack(){

        m_world->set_gravity(b3Vec3r(0, 0, -10));

        // create a dynamic body
        b3Transr pose, velocity;

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;

        b3Vec3r p(0, 0, 30);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(0, 0, -10);
        b3Vec3r w(0, 0, 0);

        for(int i = 0;i<num;i++){
            p = p - b3Vec3r(0,0,2);
            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* b = m_world->create_body(body_def);
            utils.track_body(b, "init_velocity_sphere");

            b3Fixture* f = b->create_fixture(fixture_def);
            utils.track_fixture(f, "init_velocity_sphere");
        }

        // Create a static ground plane
        {
            // create a ground
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            // Set the shape and density for the fixture
            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;

            // Create the fixture and attach it to the ground body
            b3Fixture* fg = ground_body->create_fixture(fixture_def);

            // Track the ground body and fixture for debugging or analysis
            utils.track_body(ground_body, "ground");
            utils.track_fixture(fg, "ground");

        }
    }

    static Test* create() {
        return new TestSphereStack;
    };

};

static int test_index = register_test("Sphere Scene Test", "Sphere Stack", TestSphereStack::create);
