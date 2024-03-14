
#include "scene_test.hpp"

class TestGroundView : public SceneTestBase {

public:

  TestGroundView() {
	m_world->set_gravity(b3Vector3d(0, 0, 0));

	b3TransformD pose, velocity;
	pose.set_linear(0, 0, -1);

	b3BodyDef body_def;
	body_def.m_type = b3BodyType::b3_static_body;
	body_def.set_initial_status(pose, velocity);
	b3Body *plane = m_world->create_body(body_def);

	b3PlaneShape plane_shape;
	plane_shape.set_as_plane(10, 10);

	b3FixtureDef fixture_def;
	fixture_def.m_shape = &plane_shape;
	fixture_def.m_density = 0;

	plane->create_fixture(fixture_def);
  }

  static TestBase *create() {
	return new TestGroundView;
  }
};

static int test_ground_view_index = register_test("Debug", "Ground View", TestGroundView::create);