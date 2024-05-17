
#include "test.hpp"

class TestSphereRotating : public Test {

public:

    TestSphereRotating() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));
        {
            b3BodyDef body_def;
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(b3_pi / 2, 0, 0);
            body_def.set_init_velocity(v, w);
            body_def.m_type = b3BodyType::b3_static_body;

            b3Body* body = m_world->create_body(body_def);

            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(1);

            b3FixtureDef fixture_def;
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 0;
            fixture_def.m_restitution = 0;
            fixture_def.m_friction = 1;

            body->create_fixture(fixture_def);
        }

        {
            b3BodyDef body_def;
            b3Vec3r p(0, 0, 1.2);
            b3Vec3r q(0, 0, 0);
            body_def.set_init_pose(p, q);
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Body* body = m_world->create_body(body_def);

            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(0.2);

            b3FixtureDef fixture_def;
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1;
            fixture_def.m_restitution = 0;
            fixture_def.m_friction = 1;

            body->create_fixture(fixture_def);
        }
    }

    static Test* create() {
        return new TestSphereRotating;
    }
};

static int test_index = register_test("Sphere Scene Test", "Sphere Rotating", TestSphereRotating::create);