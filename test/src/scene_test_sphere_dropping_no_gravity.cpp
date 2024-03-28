
#include "scene_test.hpp"


class TestSphereDroppingNoGravity : public SceneTestBase {

public:

	TestSphereDroppingNoGravity() {

		m_world->set_gravity(b3Vector3r(0, 0, 0));
		int num_of_spheres = 3;

		// create a dynamic body
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

		//create a series of touching spheres 

		for (int i = 0; i < num_of_spheres; i++) {
			for (int j = 0; j < 1; j++) {

                b3Vector3r p(0, 1.0 * j, 1.0 * i + 10);
                b3Vector3r q(0, 0, 0);
                b3Vector3r v(0, 0, -5);
                b3Vector3r w(0, 0, 0);

                body_def.set_init_pose(p, q);
                body_def.set_init_velocity(v, w);

				m_world->create_body(body_def)->create_fixture(fixture_def);
			}
		}

		// create a ground
        b3Vector3r p(0, 0, 0);
        b3Vector3r q(0, 0, 0);
        b3Vector3r v(0, 0, 0);
        b3Vector3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
		body_def.m_type = b3BodyType::b3_static_body;
		b3Body* ground_body = m_world->create_body(body_def);

		b3PlaneShape ground_shape;
		ground_shape.set_as_plane(50, 50);

		fixture_def.m_shape = &ground_shape;
		fixture_def.m_density = 0;

		ground_body->create_fixture(fixture_def);

		p = {0, 0, 15};
		q = {0, b3_pi, 0};
		body_def.set_init_pose(p, q);
		ground_body = m_world->create_body(body_def);
		ground_body->create_fixture(fixture_def);
	}

	static TestBase* create() {
		return new TestSphereDroppingNoGravity;
	}
};

static int test_index = register_test("Sphere Scene Test", "Sphere Stack Dropping No Gravity",
									  TestSphereDroppingNoGravity::create);
