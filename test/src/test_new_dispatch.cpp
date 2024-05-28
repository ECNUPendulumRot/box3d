
#include "test.hpp"


class TestNewDispatch : public Test {

public:

    TestNewDispatch() {
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0,  -4.0f, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 2, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* sphere = m_world->create_body(body_def);

            b3SphereShape shape;
            shape.set_as_sphere(1.0);

            b3FixtureDef box_fd;
            box_fd.m_shape = &shape;
            box_fd.m_friction = 0.0;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            sphere->create_fixture(box_fd);
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0,  4.0f, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, -2, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* sphere = m_world->create_body(body_def);

            b3SphereShape shape;
            shape.set_as_sphere(1.0);

            b3FixtureDef box_fd;
            box_fd.m_shape = &shape;
            box_fd.m_friction = 0.0;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            sphere->create_fixture(box_fd);
        }
    }

    static Test* create() {
        return new TestNewDispatch();
    }
};

static int test_index = register_test("Dispatch", "test",TestNewDispatch::create);