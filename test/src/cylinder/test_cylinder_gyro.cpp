
#include "test.hpp"
#include "gl_render_points.hpp"

#include <iostream>

class TestCylinderGyro : public Test {

    b3Body* m_gyro_body = nullptr;
    b3Body* m_fixed_body = nullptr;

public:

    TestCylinderGyro() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_dynamic_body;
            /// add damping
            bodyDef.m_linear_damping = 0;
            bodyDef.m_angular_damping = 0;

            b3Vec3r p(0, -1, 1.6);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v;
            b3Vec3r w(0, 100, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            m_gyro_body = m_world->create_body(bodyDef);

            b3CylinderShape shape1;
            shape1.set_as_cylinder(0.5, 0.04);
            // shape.set_centroid(b3Vec3r(0, 0, -0.1));

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape1;
            fixtureDef.m_density = 1;
            b3Vec3r p1(0, 0, 0);
            b3Quaternionr q1(b3Vec3r(-b3_pi_half, 0, 0));
            fixtureDef.m_local_transform.set(p1, q1);
            fixtureDef.m_spinning_friction = 0.2;
            fixtureDef.m_rolling_friction = 0.2;

            m_gyro_body->create_fixture(fixtureDef);

            b3CylinderShape shape2;
            shape2.set_as_cylinder(0.01, 2);
            fixtureDef.m_shape = &shape2;
            fixtureDef.m_density = 0;
            b3Vec3r p2(0, 0, 0);
            b3Quaternionr q2(b3Vec3r(-b3_pi_half, 0, 0));
            fixtureDef.m_local_transform.set(p2, q2);

            m_gyro_body->create_fixture(fixtureDef);
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
            // b3Vec3r pivotInB(0, 0, -1);
            b3Vec3r pivotInB(0, 1, 0);
            b3Point2PointConstraint* constraint = new b3Point2PointConstraint(m_fixed_body, m_gyro_body,
                                                                              pivotInA, pivotInB, false);
            m_world->add_constraint(constraint);
        }
    }

    void step(Settings& settings) override {
        Test::step(settings);
        auto p = m_gyro_body->get_world_transform().position();
        auto w = m_gyro_body->get_angular_velocity();
        // std::cout << "pos: " << p.x << " " << p.y << " " << p.z << std::endl;
        std::cout << "angular velocity: " << w.x << " " << w.y << " " << w.z << std::endl;
        g_debug_draw.m_points->permanent_vertex(p, b3Color(0, 0, 1), 3.0);
    }

    static Test* create() {
        return new TestCylinderGyro;
    }
};


static int test_index = register_test("Cylinder", "gyro", TestCylinderGyro::create);