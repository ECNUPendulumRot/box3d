
#include "test.hpp"

class SphereCradleTest : public Test {

    int sphere_count = 3;

public:

    SphereCradleTest() {

        m_world->set_gravity(b3Vec3r(0, 0, -10));
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
            b3Fixture* fg = ground->create_fixture(fixture_def);

            utils.track_body(ground, "ground");
            utils.track_fixture(fg, "ground");
        }
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, -3, 1);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 5, 0);
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

            b3Body* b = m_world->create_body(body_def);
            utils.track_body(b, "init_velocity_sphere");

            b3Fixture* f = b->create_fixture(fixture_def);
            utils.track_fixture(f, "init_velocity_sphere");


            v.set_zero();
            body_def.set_init_velocity(v, w);
            for(int i = 1; i <= sphere_count; i++) {
                b3Vec3r position(0, 1 + 2 * i, 1);
                body_def.set_init_pose(position, q);

                b3Body* bi  = m_world->create_body(body_def);
                b3Fixture* fi = bi->create_fixture(fixture_def);

                utils.track_body(bi, ("sphere_" + std::to_string(i)).c_str());
                utils.track_fixture(fi, ("sphere_" + std::to_string(i)).c_str());

            }
        }
    }

    static Test* create() {
        return new SphereCradleTest();
    }
};

static int test_index = register_test("Sphere Scene Test", "Cradle", SphereCradleTest::create);