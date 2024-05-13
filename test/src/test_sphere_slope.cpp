
#include "test.hpp"


class TestSphereSlope : public Test {

public:

    TestSphereSlope() {
        m_world->set_gravity(b3Vec3r(0, 0, -10.0));


        b3BodyDef ground_def;
        b3Vec3r p(0, 0, 0);
        b3Vec3r q(b3_pi / 6, 0, 0);

        ground_def.set_init_pose(p, q);

        ground_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(ground_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(50, 5);

        b3FixtureDef ground_fd;
        ground_fd.m_restitution = 0.0;
        ground_fd.m_friction = 1.0;
        ground_fd.m_density = 0.0;
        ground_fd.m_shape = &ground_shape;

        ground_body->create_fixture(ground_fd);

        b3Transformr ground_transform(p, b3Quaternionr(q));
        p = ground_transform.transform(b3Vec3r(0, 0 , 1));
        spdlog::info("p: {}, {}, {}", p.x, p.y, p.z);

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        q.set(0, 0, 0);
        b3Vec3r v(0, 0, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        b3Body* sphere1 = m_world->create_body(body_def);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1);

        b3FixtureDef sphere_fd;
        sphere_fd.m_restitution = 0;
        sphere_fd.m_friction = 1;
        sphere_fd.m_density = 1.0;
        sphere_fd.m_shape = &sphere_shape;

        sphere1->create_fixture(sphere_fd);

    }


    static Test* create() {
        return new TestSphereSlope();
    }
};


static int test_index = register_test("Sphere Scene Test", "Sphere Slope", TestSphereSlope::create);