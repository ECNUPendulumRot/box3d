
#include "scene_test.hpp"


class TestSphereCollide: public SceneTestBase {

public:

    TestSphereCollide() {

        m_world->set_gravity(b3Vector3d(0, 0, 0));

        // create a dynamic body
        b3TransformD pose, velocity;
        pose.set_linear(b3Vector3d(0, -5, 0));
        velocity.set_linear(b3Vector3d(0, 10, 0));

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;
        body_def.set_initial_status(pose, velocity);
        b3Body* sphere1 = m_world->create_body(body_def);

        pose.set_linear(b3Vector3d(0, 5, 0));
        velocity.set_linear(b3Vector3d(0, 0, 0));
        body_def.set_initial_status(pose, velocity);
        b3Body* sphere2 = m_world->create_body(body_def);

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(0.5);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0.3;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;

        sphere1->create_fixture(fixture_def);
        sphere2->create_fixture(fixture_def);
    }

    static TestBase* create() {
        return new TestSphereCollide;
    }
};

static int test_index = register_scene_test("Debug", "sphere collide", TestSphereCollide::create);
