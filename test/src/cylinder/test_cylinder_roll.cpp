
#ifndef TEST_CYLINDER_HPP
#define TEST_CYLINDER_HPP

#include "test.hpp"

class TestCylinderRoll : public Test {

public:

    TestCylinderRoll() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, 0, 0.5);
            b3Vec3r q(-b3_pi_half, 0, 0);
            b3Vec3r v(2, 0, 0);
            b3Vec3r w(0, 4, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            b3CylinderShape shape;
            shape.set_as_cylinder(0.5, 0.1);

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 1;
            fixtureDef.m_friction = 0.2;
            b3Vec3r p1(0, 0, 0);
            b3Quaternionr q1(b3Vec3r(0, 0, 0));
            fixtureDef.m_local_transform.set(p1, q1);

            b3Body* cylinderBody = m_world->create_body(bodyDef);
            cylinderBody->create_fixture(fixtureDef);
        }
        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_static_body;
            b3Vec3r p(0, 0, -1);
            b3Vec3r q(0, 0, 0);
            bodyDef.set_init_pose(p, q);

            b3Body* fixedBody = m_world->create_body(bodyDef);

            b3FixtureDef fixtureDef;
            fixtureDef.m_friction = 0.2f;

            b3CubeShape shape;
            shape.set_as_box(10, 10, 1);

            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 0;

            fixedBody->create_fixture(fixtureDef);
        }
    }

    static Test* create() {
        return new TestCylinderRoll;
    }
};


static int test_index = register_test("Cylinder", "Cylinder Roll", TestCylinderRoll::create);

#endif