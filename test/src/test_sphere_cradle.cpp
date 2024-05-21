
#include "test.hpp"

class SphereCradleTest : public Test {

    int sphere_count = 2;

    // has init velocity sphere body.
    b3Body* m_dynamic_sphere;

    b3Body* m_bodies[2048];

public:

    SphereCradleTest() {
        m_world->set_gravity(b3Vec3r::zero());
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_static_body;

            b3PlaneShape plane_shape;
            plane_shape.set_as_plane(100, 100);

            b3FixtureDef fixture_def;
            fixture_def.m_shape = &plane_shape;
            fixture_def.m_density = 0.0;
            fixture_def.m_restitution = 1.0;
            fixture_def.m_friction = 0.0;

            b3Body* ground = m_world->create_body(body_def);
            ground->create_fixture(fixture_def);
        }
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, -3, 1);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            // create a sphere shape
            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(1.0);

            // create a fixture definition
            b3FixtureDef fixture_def;
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_friction = 0;
            fixture_def.m_restitution = 1.0;
            fixture_def.m_density = 1.0;

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);
            m_dynamic_sphere = m_world->create_body(body_def);

            m_dynamic_sphere->create_fixture(fixture_def);

            v.set_zero();
            body_def.set_init_velocity(v, w);
            for(int i = 0; i < sphere_count; i++) {
                b3Vec3r position(0, 3 + 2 * i, 1);
                body_def.set_init_pose(position, q);

                m_bodies[i] = m_world->create_body(body_def);
                m_bodies[i]->create_fixture(fixture_def);
            }
        }

    }

    static Test* create() {
        return new SphereCradleTest();
    }
};

static int test_index = register_test("Sphere Scene Test", "Cradle", SphereCradleTest::create);