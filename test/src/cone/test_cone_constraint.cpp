
#include "test.hpp"
#include "gl_render_points.hpp"

#include "spdlog/spdlog.h"

class TestConeConstraint :public Test {

    b3Body* m_fixedBody;
    b3Body* m_gyroBody;

    b3Point2PointConstraint* constraint = nullptr;

public:

    TestConeConstraint() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));
        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_static_body;
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);
            b3Vec3r p(0, 0, 0);
            b3Vec3r q(0, 0, 0);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);
            m_fixedBody = m_world->create_body(bodyDef);

            b3CubeShape shape;
            shape.set_as_box(2, 2, 1);
            b3FixtureDef fixtureDef;
            fixtureDef.m_local_transform.set_identity();
            fixtureDef.m_shape = &shape;
            fixtureDef.m_density = 0;

            m_fixedBody->create_fixture(fixtureDef);
        }
        {
            b3BodyDef bodyDef;
            bodyDef.m_type = b3BodyType::b3_dynamic_body;
            /// add damping
            bodyDef.m_linear_damping = 0.01;
            bodyDef.m_angular_damping = 0.01;

            b3Vec3r p(0, -0.5, 1.f + 0.5f * sqrtf(3));
            b3Vec3r q(b3_pi_6, 0, 0);
            b3Vec3r v;
            b3Vec3r w(0, 0, 15);
            bodyDef.set_init_pose(p, q);
            bodyDef.set_init_velocity(v, w);

            m_gyroBody = m_world->create_body(bodyDef);

            b3CylinderShape shape1;
            shape1.set_as_cylinder(0.5, 0.04);
            // shape.set_centroid(b3Vec3r(0, 0, -0.1));

            b3FixtureDef fixtureDef;
            fixtureDef.m_shape = &shape1;
            fixtureDef.m_density = 1;
            b3Vec3r p1(0, 0, 0);
            b3Quaternionr q1(b3Vec3r(0, 0, 0));
            fixtureDef.m_local_transform.set(p1, q1);
            fixtureDef.m_spinning_friction = 0.2;
            fixtureDef.m_rolling_friction = 0.2;

            m_gyroBody->create_fixture(fixtureDef);

            b3CylinderShape shape2;
            shape2.set_as_cylinder(0.01, 2);
            fixtureDef.m_shape = &shape2;
            fixtureDef.m_density = 0;
            b3Vec3r p2(0, 0, 0);
            b3Quaternionr q2(b3Vec3r(0, 0, 0));
            fixtureDef.m_local_transform.set(p2, q2);

            m_gyroBody->create_fixture(fixtureDef);
        }
        {
            b3Vec3r pivotInA(0, 0, 1);
            b3Vec3r pivotInB(0, 0, -1);
            constraint = new b3Point2PointConstraint(m_fixedBody, m_gyroBody,
                                             pivotInA, pivotInB, true);
            constraint->set_apply_force_direction(b3Vec3r(0, 0, 0));
            m_world->add_constraint(constraint);
        }
    }

    void step(Settings& settings) override {
        b3Vec3r conePos = m_gyroBody->get_position();
        b3Vec3r apply_point = conePos - b3Vec3r(0, 0, 1);
        constraint->set_apply_force_direction(apply_point);

        Test::step(settings);

        spdlog::info("pos: {}, {}, {}", conePos.x, conePos.y, conePos.z);

        g_debug_draw.m_points->permanent_vertex(conePos, b3Color(1, 0, 0), 2.0);
    }

    static Test* create() {
        return new TestConeConstraint;
    }
};

static int test_index = register_test("Cone", "cone constraint", TestConeConstraint::create);