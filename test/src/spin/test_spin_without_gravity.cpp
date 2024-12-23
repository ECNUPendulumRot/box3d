
#include "test.hpp"
#include "dynamics/constraint/b3_point_point_constraint.hpp"

#include <iostream>

class TestSpinWithoutGravity : public Test {

    b3Body* spinBody = nullptr;
    b3Body* fixedBody = nullptr;

public:

    TestSpinWithoutGravity() {
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, 0, 3);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 20, 0);
            b3Vec3r w(0, 0, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            b3SphereShape shape;
            shape.set_as_sphere(0.2);

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 1;

            spinBody = m_world->create_body(bodyDef);
            spinBody->create_fixture(fixtureDef);
        }
        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_static_body;
            b3Vec3r p(0, 0, 0);
            b3Vec3r q(0, 0, 0);
            bodyDef.set_init_pose(p, q);

            fixedBody = m_world->create_body(bodyDef);

            b3FixtureDef fixtureDef;

            b3SphereShape shape;
            shape.set_as_sphere(0.6);

            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 0;

            fixedBody->create_fixture(fixtureDef);
        }

        {
            b3Vec3r pivotInA(0, 0, 0);
            b3Vec3r pivotInB(0, 0, -3);
            b3Point2PointConstraint* constraint = new b3Point2PointConstraint(fixedBody, spinBody, pivotInA, pivotInB);
            m_world->add_constraint(constraint);
        }
    }

    virtual void step(Settings& settings) override {
        Test::step(settings);
        auto v = spinBody->get_linear_velocity();
        auto w = spinBody->get_angular_velocity();
        std::cout << v.x << " " << v.y << " " << v.z << " " << w.x << " " << w.y << " " << w.z << std::endl;
    }

    static Test* create() {
        return new TestSpinWithoutGravity;
    }
};


static int test_index = register_test("Spin", "Spin without gravity", TestSpinWithoutGravity::create);