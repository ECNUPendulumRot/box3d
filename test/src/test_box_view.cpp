
#include "scene_test.hpp"

class TestBoxView: public SceneTestBase {

public:

    TestBoxView() {

        m_world->set_gravity(b3Vector3d(0, 0, 0));

        // create a dynamic body
        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;
        b3Body* cube1 = m_world->create_body(body_def);

        b3TransformD pose, velocity;
        pose.set_linear({0, -5, 0});
        pose.set_angular({0, 1.2, -0.8});
        velocity.set_linear(b3Vector3d(0, 5, 0));
        body_def.set_initial_status(pose, velocity);
        b3Body* cube2 = m_world->create_body(body_def);

        // create a cube shape
        b3CubeShape cube_shape;
        cube_shape.set_as_box(1.0, 1.0, 1.0);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &cube_shape;
        fixture_def.m_friction = 0.3;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;

        cube1->create_fixture(fixture_def);
        cube2->create_fixture(fixture_def);
    }

    static TestBase* create() {
        return new TestBoxView;
    }
};

static int test_index = register_test("Debug", "box view", TestBoxView::create);

