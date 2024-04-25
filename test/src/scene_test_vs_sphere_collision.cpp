
#include "scene_test.hpp"

class TestVSPlaneSphereCollide : public SceneTestBase {

public:
    TestVSPlaneSphereCollide() {
        m_world->set_gravity(b3Vector3r(0, 0, -10));

        // create a sphere body
        b3Transformr pose, velocity;

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        b3Vector3r p(0, 0, 5);
        b3Vector3r q(0, 0, 0);
        b3Vector3r v(0, 0, 0);
        b3Vector3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        b3Body *sphere_body = m_world->create_body(body_def);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(0.5);

        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_density = 1.0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_friction = 0;

        sphere_body->create_fixture(fixture_def);

        // create a ground
        p = {0, 0, 0};
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body *ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(10, 10);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        ground_body->create_fixture(fixture_def);
    }

    void step() override {
        m_world->step(1.0 / 60, 8, 8);
    }

    static TestBase *create() {
        return new TestVSPlaneSphereCollide();
    }
};


static int test_plane_sphere_collide = register_test("Velocity Solver Test", "Sphere Colllision",
                                                     TestVSPlaneSphereCollide::create);