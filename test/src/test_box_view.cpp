
#include "scene_test.hpp"

class TestBoxView : public SceneTestBase {

public:

  TestBoxView() {
    // create a dynamic body
    b3BodyDef body_def;
    body_def.m_type = b3BodyType::b3_dynamic_body;
    b3Body *cube = m_world->create_body(body_def);

    // create a cube shape
    b3CubeShape cube_shape;
    cube_shape.set_as_box(3.0, 3.0, 3.0);

    // create a fixture definition
    b3FixtureDef fixture_def;
    fixture_def.m_shape = &cube_shape;
    fixture_def.m_friction = 0.3;
    fixture_def.m_restitution = 1.0;
    fixture_def.m_density = 1.0;

    cube->create_fixture(fixture_def);
  }

  static TestBase *create() {
	  return new TestBoxView;
  }
};

static int test_index = register_test("Debug", "box view", TestBoxView::create);

