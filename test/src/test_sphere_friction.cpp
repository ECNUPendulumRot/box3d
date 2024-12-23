
#include "test.hpp"

class TestSphereFriction : public Test {

public:

    TestSphereFriction() {
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, -2, 2);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* sphere1 = m_world->create_body(body_def);

            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(1);

            b3FixtureDef sphere_fd;
            sphere_fd.m_restitution = 1;
            sphere_fd.m_friction = 0.4;
            sphere_fd.m_density = 1.0;
            sphere_fd.m_shape = &sphere_shape;

            sphere1->create_fixture(sphere_fd);
        }

        {
            b3BodyDef ground_def;
            ground_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(ground_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            b3FixtureDef ground_fd;
            ground_fd.m_restitution = 1.0;
            ground_fd.m_friction = 0.4;
            ground_fd.m_density = 0.0;
            ground_fd.m_shape = &ground_shape;

            ground_body->create_fixture(ground_fd);
        }
    }

    static Test* create() {
        return new TestSphereFriction();
    }

};

static int test_index = register_test("Sphere Scene Test", "Sphere Friction", TestSphereFriction::create);