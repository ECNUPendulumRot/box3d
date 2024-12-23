
#include "test.hpp"


class TestConeRotation : public Test {

public:

    TestConeRotation() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3BodyDef bodyDef;
        bodyDef.m_type = b3BodyType::b3_dynamic_body;

        b3Vec3r p(0, 0, 2.5);
        b3Vec3r q(0, b3_pi, 0);
        b3Vec3r v;
        b3Vec3r w(0, 0, 20);
        bodyDef.set_init_pose(p, q);
        bodyDef.set_init_velocity(v, w);

        b3ConeShape shape;
        shape.set_as_cone(2, 5);

        b3FixtureDef fixtureDef;
        fixtureDef.m_shape = &shape;
        fixtureDef.m_density = 1;
        fixtureDef.m_friction = 0.2;
        fixtureDef.m_spinning_friction = 0.2;
        fixtureDef.m_rolling_friction = 0.2;

        b3Body* body = m_world->create_body(bodyDef);
        body->create_fixture(fixtureDef);

        {
            b3BodyDef groundBodyDef;
            groundBodyDef.m_type = b3BodyType::b3_static_body;

            b3Body* body = m_world->create_body(groundBodyDef);

            b3FixtureDef ground_fixtureDef;

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(20, 20);

            ground_fixtureDef.m_shape = &ground_shape;
            ground_fixtureDef.m_density = 0;
            ground_fixtureDef.m_friction = 0.2;
            ground_fixtureDef.m_spinning_friction = 0.2;
            ground_fixtureDef.m_rolling_friction = 0.2;


            body->create_fixture(ground_fixtureDef);
        }
    }

    static Test* create() {
        return new TestConeRotation;
    }
};


static int test_index = register_test("Cone", "rotate", TestConeRotation::create);