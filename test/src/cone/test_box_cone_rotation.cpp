
#include "test.hpp"


class TestBoxConeRotation : public Test {

public:

    TestBoxConeRotation() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3BodyDef bodyDef;
        bodyDef.m_type = b3BodyType::b3_dynamic_body;

        b3Vec3r p(0, 0, 0.6);
        b3Vec3r q(0, b3_pi, 0);
        b3Vec3r v;
        b3Vec3r w(0, 0, 200);
        bodyDef.set_init_pose(p, q);
        bodyDef.set_init_velocity(v, w);

        b3ConeShape shape;
        shape.set_as_cone(0.5, 1.2);

        b3FixtureDef fixtureDef;
        fixtureDef.m_shape = &shape;
        fixtureDef.m_density = 1;
        fixtureDef.m_friction = 0.2;
        fixtureDef.m_restitution = 0;
        fixtureDef.m_spinning_friction = 0.4;
        fixtureDef.m_rolling_friction = 0.4;

        b3Body* body = m_world->create_body(bodyDef);
        body->create_fixture(fixtureDef);

        {
            b3BodyDef boxBodyDef;
            boxBodyDef.m_type = b3BodyType::b3_static_body;
            b3Vec3r boxPos(0, 0, -10);
            b3Vec3r boxq(0, 0, 0);
            boxBodyDef.set_init_pose(boxPos, boxq);

            b3Body* body = m_world->create_body(boxBodyDef);

            b3FixtureDef boxFixtureDef;

            b3CubeShape boxShape;
            boxShape.set_as_box(20, 20, 10);

            boxFixtureDef.m_shape = &boxShape;
            boxFixtureDef.m_density = 0;
            boxFixtureDef.m_friction = 0.2;
            boxFixtureDef.m_restitution = 0;
            boxFixtureDef.m_spinning_friction = 0.8;
            boxFixtureDef.m_rolling_friction = 0.8;


            body->create_fixture(boxFixtureDef);
        }
    }

    static Test* create() {
        return new TestBoxConeRotation;
    }
};


static int test_index = register_test("Cone", "box cone rotate", TestBoxConeRotation::create);