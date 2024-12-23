
#include "test.hpp"

#include <iostream>

class TestTwoCone : public Test {

    b3Body* twoConeBody;

    b3Body* fixedBody;

public:

    TestTwoCone() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_dynamic_body;
            bodyDef.m_linear_damping = 0.1;
            bodyDef.m_angular_damping = 0.1;

            b3Vec3r p(0, 0, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0,0);
            b3Vec3r w(0, 100,0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            twoConeBody = m_world->create_body(bodyDef);
        }

        {
            b3ConeShape shape;
            shape.set_as_cone(1, 1);

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 1;

            b3Vec3r p(0, -0.5, 0);
            b3Quaternionr q(b3Vec3r(-b3_pi_half, 0, 0));
            fixtureDef.m_local_transform.set(p, q);

            twoConeBody->create_fixture(fixtureDef);
        }

        {
            b3ConeShape shape;
            shape.set_as_cone(1, 1);

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 1;

            b3Vec3r p(0, 0.5, 0);
            b3Quaternionr q(b3Vec3r(b3_pi_half, 0, 0));
            fixtureDef.m_local_transform.set(p, q);

            twoConeBody->create_fixture(fixtureDef);
        }
        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_static_body;

            b3Vec3r p(0, 0, 2);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            fixedBody = m_world->create_body(bodyDef);

            b3CubeShape shape;
            shape.set_as_box(0.01, 0.01, 2);

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 0;

            fixedBody->create_fixture(fixtureDef);
        }

        {
            b3Vec3r pivotInFixedBody(0, 0, -2);
            b3Vec3r pivotInTwoConeBody(0, 0, 0);
            auto constraint = new b3Point2PointConstraint(fixedBody, twoConeBody,
              pivotInFixedBody, pivotInTwoConeBody, false);

            m_world->add_constraint(constraint);
        }
    }

    virtual void step(Settings& settings) override {
        Test::step(settings);
        auto v = twoConeBody->get_linear_velocity();
        auto w = twoConeBody->get_angular_velocity();
        std::cout << v.x << " " << v.y << " " << v.z << " " << w.x << " " << w.y << " " << w.z << std::endl;

    }

    static Test* create() {
        return new TestTwoCone;
    }
};


static int test_index = register_test("Cone", "Two Cone", TestTwoCone::create);