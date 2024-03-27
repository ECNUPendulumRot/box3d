#include "scene_test.hpp"

class TestNewtonsCradle :public SceneTestBase {
public:
	TestNewtonsCradle() {

        m_world->set_gravity(b3Vector3r(0, 0, 0));
        int num_of_spheres = 3;
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


        pose.set_linear(b3Vector3r(0, 0, 0.5));
        velocity.set_linear(b3Vector3r(0, 5, 0));
        body_def.set_initial_status(pose, velocity);
        m_world->create_body(body_def)->create_fixture(fixture_def);

        real x = 2;
       
        velocity.set_linear(b3Vector3r(0, 0, 0));
        for (int32 i = 0; i < num_of_spheres - 1; i++) {
            pose.set_linear(b3Vector3r(0, x + i, 0.5));
            body_def.set_initial_status(pose, velocity);
            m_world->create_body(body_def)->create_fixture(fixture_def);
        }
        

        // create a ground
        pose.set_linear(0, 0, 0);
        body_def.set_initial_status(pose, velocity);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(50, 50);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        ground_body->create_fixture(fixture_def);
		
	}

    static TestBase* create() {
        return new TestNewtonsCradle;
    }
};

static int test_index = register_test("Sphere Scene Test", "Newton's Cradle", TestNewtonsCradle::create);