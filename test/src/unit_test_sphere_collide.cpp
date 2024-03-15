
#include "scene_test.hpp"


class TestSphereCollide : public SceneTestBase {

public:

  TestSphereCollide() {
    m_world->set_gravity(b3Vector3d(0, 0, -10));

    // create a dynamic body
    b3TransformD pose, velocity;
    pose.set_linear(b3Vector3d(0, 0, 0.5));

    b3BodyDef body_def;
    body_def.m_type = b3BodyType::b3_dynamic_body;
    body_def.set_initial_status(pose, velocity);
    b3Body *sphere1 = m_world->create_body(body_def);

    pose.set_linear(b3Vector3d(0, 0, 2));
    body_def.set_initial_status(pose, velocity);
    // b3Body* sphere2 = m_world->create_body(body_def);

    // create a sphere shape
    b3SphereShape sphere_shape;
    sphere_shape.set_as_sphere(0.5);

    // create a fixture definition
    b3FixtureDef fixture_def;
    sphere_shape.set_color(b3Vector3d(1, 0, 0));
    fixture_def.m_shape = &sphere_shape;
    fixture_def.m_friction = 0.3;
    fixture_def.m_restitution = 1.0;
    fixture_def.m_density = 1.0;

    sphere1->create_fixture(fixture_def);
    sphere_shape.set_color(b3Vector3d(0, 1, 0));
    // sphere2->create_fixture(fixture_def);

    b3PlaneShape ground_shape;
    ground_shape.set_as_plane(10, 10);
    fixture_def.m_shape = &ground_shape;
    fixture_def.m_density = 0;
    pose.set_linear(0, 0, 0);
    body_def.set_initial_status(pose, velocity);
    body_def.m_type = b3BodyType::b3_static_body;
    m_world->create_body(body_def)->create_fixture(fixture_def);
  }

  static TestBase *create() {
	  return new TestSphereCollide;
  }
};

static int test_index = register_test("Debug", "Spheres Collide", TestSphereCollide::create);
