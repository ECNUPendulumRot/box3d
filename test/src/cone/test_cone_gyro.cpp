
#include "test.hpp"
#include "dynamics/constraint/b3_point_point_constraint.hpp"

class TestConeGyro : public Test {

    b3Body* m_cone_body;
    b3Body* m_fixed_body;

public:

    TestConeGyro() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_dynamic_body;
            /// add damping
            // bodyDef.m_linear_damping = 0.01;
            // bodyDef.m_angular_damping = 0.01;

            b3Vec3r p(0, -0.5, 1.6);
            b3Vec3r q(b3_pi_half, 0, 0);
            b3Vec3r v;
            b3Vec3r w(0, 200, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            b3ConeShape shape;
            shape.set_as_cone(0.1, 0.2);
            // shape.set_centroid(b3Vec3r(0, 0, -0.1));

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 1;
//            fixtureDef.m_spinning_friction = 0.2;
//            fixtureDef.m_rolling_friction = 0.2;

            m_cone_body = m_world->create_body(bodyDef);
            m_cone_body->create_fixture(fixtureDef);
        }
        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_static_body;

            b3Vec3r p(0, 0, 2);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v;
            b3Vec3r w(0, 0, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            b3CubeShape shape;
            shape.set_as_box(0.01, 0.01, 0.4);

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 0;

            m_fixed_body = m_world->create_body(bodyDef);
            m_fixed_body->create_fixture(fixtureDef);
        }
        {
            b3Vec3r pivotInA(0, 0, -0.4);
            b3Vec3r pivotInB(0, 0, -0.5);
            b3Point2PointConstraint* constraint = new b3Point2PointConstraint(m_fixed_body, m_cone_body, pivotInA, pivotInB);
            m_world->add_constraint(constraint);
        }
    }

    static Test* create() {
        return new TestConeGyro;
    }
};


static int test_index = register_test("Cone", "cone gyro", TestConeGyro::create);