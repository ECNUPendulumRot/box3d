
#include "scene_test.hpp"

class TestPlaneSphereCollide : public SceneTestBase {

public:
    TestPlaneSphereCollide() {
        m_world->set_gravity(b3Vector3d(0, 0, -10));

        // create a sphere body
        b3TransformD pose, velocity;
        pose.set_linear(0, 0, 5);
        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;
        body_def.set_initial_status(pose, velocity);
        b3Body* sphere_body = m_world->create_body(body_def);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(0.5);

        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_density = 1.0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_friction = 0;

        sphere_body->create_fixture(fixture_def);

        // create a ground
        pose.set_linear(0, 0, 0);
        body_def.set_initial_status(pose, velocity);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(10, 10);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        ground_body->create_fixture(fixture_def);
    }

    static TestBase* create() {
        return new TestPlaneSphereCollide();
    }
};


static int test_plane_sphere_collide = register_test("Debug", "Plane Sphere Collide", TestPlaneSphereCollide::create);