
#include "scene_test.hpp"

class TestSphereCubeCollide : public SceneTestBase {

public:

    TestSphereCubeCollide() {
	    m_world->set_gravity(b3Vector3r(0.0, 0.0, 0.0));

	// create a dynamic body

	    b3BodyDef body_def;
	    body_def.m_type = b3BodyType::b3_dynamic_body;

        b3Vector3r p(0, -2, 0);
        b3Vector3r q(0, 0, 0);
        b3Vector3r v(0, 10, 0);
        b3Vector3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

	    b3Body *sphere = m_world->create_body(body_def);

	    p = {0, 2, 0};
	    v = {0, 0, 0};

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

	    b3Body *cube = m_world->create_body(body_def);

	    b3SphereShape sphere_shape;
	    sphere_shape.set_as_sphere(0.5);
	    b3CubeShape cube_shape;
	    cube_shape.set_as_box(0.5, 0.5, 0.5);

	    b3FixtureDef fixture_def;
	    fixture_def.m_shape = &sphere_shape;
	    fixture_def.m_friction = 0.3;
	    fixture_def.m_restitution = 1.0;
	    fixture_def.m_density = 1.0;

	    sphere->create_fixture(fixture_def);

	    fixture_def.m_shape = &cube_shape;
	    cube->create_fixture(fixture_def);
    }

  static TestBase *create() {
	return new TestSphereCubeCollide();
  }
};

static int test_sphere_cube_collide = register_test("Cube Scene Test", "Sphere Cube Collide", TestSphereCubeCollide::create);