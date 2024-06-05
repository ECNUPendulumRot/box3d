#include "test.hpp"

class TestInnerBounding :public Test {

    int c_count = 8;

    real radius = 0.5;

public:

    TestInnerBounding() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

        b3FixtureDef fixture_def;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(radius);
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(c_count * radius, -5, c_count * radius);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 10, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* b = m_world->create_body(body_def);

            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1.0;

            b3Fixture* f = b->create_fixture(fixture_def);

            utils.track_body(b, "init_velocity_sphere");
            utils.track_fixture(f, "init_velocity_sphere");
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            int index = 1;

            b3Vec3r q(0, 0, 0);
            for(int i = 0; i < c_count; i++) {
                b3Vec3r left_position(radius, radius + 2 * i * radius, radius);
                for(int j = 0; j < c_count; j++) {
                    for (int k = 0; k < c_count; k++) {
                        b3Vec3r p = left_position + 2 * j * b3Vec3r(radius, 0, 0) + 2 * k * b3Vec3r(0, 0, radius);
                        body_def.set_init_pose(p, q);
                        body_def.set_init_velocity(b3Vec3r(0, 0, 0), b3Vec3r(0, 0, 0));
                        b3Body* b = m_world->create_body(body_def);
                        b3Fixture* f = b->create_fixture(fixture_def);

                        //utils.track_body(b, ("sphere_" + std::to_string(index)).c_str());
                        utils.track_fixture(f, ("sphere_" + std::to_string(index)).c_str());
                        index++;
                    }
                }
            }
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(c_count * radius, c_count * radius, c_count * radius);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* b = m_world->create_body(body_def);
            sphere_shape.set_as_sphere(20);
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1.0;

            b3Fixture* f = b->create_fixture(fixture_def);

//            utils.track_body(b, "init_velocity_sphere");
//            utils.track_fixture(f, "init_velocity_sphere");
        }

//        b3Vec3r p;
//        b3Vec3r q;
//        {
//            b3BodyDef body_def;
//            p = { 0, 0, -10};
//            q = {0, 0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
//
//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { 0, 0, 2 * c_count * radius + 10};
//            q = {3.1415926535897, 0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
//
//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { 0, -10, 0};
//            q = {-3.1415926535897 / 2.0, 0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
//
//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { 0, 2 * radius * c_count + 10, 0};
//            q = {3.1415926535897 / 2.0, 0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
//
//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { -10, 0, 0};
//            q = {0, 3.1415926535897 / 2.0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
//
//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { 2 * radius * c_count + 10, 0, 0};
//            q = {0, -3.1415926535897 / 2.0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
    }

    static Test* create() {
        return new TestInnerBounding;
    }
};

static int test_index = register_test("Sphere Scene Test", "Inner Bounding", TestInnerBounding::create);
