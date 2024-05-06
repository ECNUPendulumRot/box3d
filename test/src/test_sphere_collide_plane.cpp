
#include "test.hpp"


class SphereCollidePlaneTest : public Test {
public:

    SphereCollidePlaneTest() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;
            b3Vec3r p(0, 0, 20);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 3, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* body = m_world->create_body(body_def);

            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(2.0);

            b3FixtureDef fixture_def;
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1.0;
            fixture_def.m_friction = 0.4;
            fixture_def.m_restitution = 0.9;

            body->create_fixture(fixture_def);
        }

        {
            // two ground planes
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_static_body;
            b3Vec3r p(0, 0, 0);
            b3Vec3r q1(b3_pi / 3, 0, 0);
            b3Vec3r q2(b3_pi * 2.0 / 3.0, 0, 0);

            body_def.set_init_pose(p, q1);
            b3Body* ground_body1 = m_world->create_body(body_def);
            body_def.set_init_pose(p, q2);
            b3Body* ground_body2 = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(50, 10);

            b3FixtureDef ground_fd;
            ground_fd.m_shape = &ground_shape;
            ground_fd.m_friction = 1.0;
            ground_fd.m_restitution = 0.0;
            ground_fd.m_density = 0.0;

            ground_body1->create_fixture(ground_fd);
            ground_body2->create_fixture(ground_fd);
        }
    }

    static Test* Create() {
        return new SphereCollidePlaneTest;
    }
};

static int test_index = register_test("Sphere Scene Test", "SphereCollidePlane", SphereCollidePlaneTest::Create);
