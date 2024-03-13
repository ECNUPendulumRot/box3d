
#include "scene_test.hpp"

class TestPlaneBoxCollide: public SceneTestBase {

public:

    TestPlaneBoxCollide() {

        m_world->set_gravity(b3Vector3d(0, 0, -10));

        // create a dynamic body
        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        b3TransformD pose, velocity;
        pose.set_linear({0, 0, 1});
        pose.set_angular({0, 0, 0});
        velocity.set_linear(b3Vector3d(0, 0, 0));
        body_def.set_initial_status(pose, velocity);

        b3Body* cube1 = m_world->create_body(body_def);

        b3CubeShape cube_shape;
        cube_shape.set_as_box(1.0, 1.0, 1.0);

        b3FixtureDef fixture_def;
        fixture_def.m_shape = &cube_shape;
        fixture_def.m_friction = 0.3;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;

        cube1->create_fixture(fixture_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(10, 10);

        b3BodyDef ground_def;
        ground_def.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_def);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        ground_body->create_fixture(fixture_def);

    }

    static TestBase* create() {
        return new TestPlaneBoxCollide;
    }
};

static int test_index = register_test("Debug", "Plane Box Collide", TestPlaneBoxCollide::create);


