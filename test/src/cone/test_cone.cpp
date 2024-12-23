
#include "test.hpp"


class TestCone : public Test {

public:

    TestCone() {
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        b3BodyDef bodyDef;
        bodyDef.m_type = b3BodyType::b3_dynamic_body;

        b3Vec3r p(0, 0, 0);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v;
        b3Vec3r w(0, 0, 0);
        bodyDef.set_init_pose(p, q);
        bodyDef.set_init_velocity(v, w);

        b3ConeShape shape;
        shape.set_as_cone(1, 1);

        b3FixtureDef fixtureDef;
        fixtureDef.m_shape = &shape;
        fixtureDef.m_density = 1;
        fixtureDef.m_spinning_friction = 0.1;

        b3Body* body = m_world->create_body(bodyDef);
        body->create_fixture(fixtureDef);
    }

    static Test* create() {
        return new TestCone;
    }
};


static int test_index = register_test("Cone", "cone", TestCone::create);