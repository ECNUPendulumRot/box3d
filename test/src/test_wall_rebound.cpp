#include "test.hpp"

class TestWallRebound :public Test {

public:

    TestWallRebound() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));
        int num_of_spheres = 5;
        // create a dynamic body
        b3Transr pose, velocity;

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(0.5);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;


        b3Vec3r p(-0.5, -0.8, 0.5);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(2, 3.2, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        b3Fixture *fg;
        b3Body * bd;

        bd = m_world->create_body(body_def);
        fg = bd->create_fixture(fixture_def);

        utils.track_body(bd, "ini_v1");
        utils.track_fixture(fg, "ini_v1");

        p = { 0.5, -0.8, 0.5 };
        v = { -2, 3.2, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        bd = m_world->create_body(body_def);
        fg = bd->create_fixture(fixture_def);

        utils.track_body(bd, "ini_v2");
        utils.track_fixture(fg, "ini_v2");

        real x = 0;


        for (int32 i = 0; i < num_of_spheres - 2; i++) {
            p = { 0, x + i, 0.5 };
            v = { 0, 0, 0 };
            //if(i==2) v={0,-3.0f,0};
            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);
            bd = m_world->create_body(body_def);
            fg = bd->create_fixture(fixture_def);

            utils.track_body(bd, ("sphere_" + std::to_string(i)).c_str());
            utils.track_fixture(fg, ("sphere_" + std::to_string(i)).c_str());
        }
        // create a ground
        p = { 0, 0, 0 };
        v = { 0, 0, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(50, 50);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        fg =  ground_body->create_fixture(fixture_def);

        utils.track_body(ground_body, "ground");
        utils.track_fixture(fg, "ground");

        p = { 0, 2.5f, 0 };
        q = {3.14159 * 0.5, 0, 0};
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        ground_body = m_world->create_body(body_def);

        fg = ground_body->create_fixture(fixture_def);

        utils.track_body(ground_body, "wall");
        utils.track_fixture(fg, "wall");
    }

    static Test* create() {
        return new TestWallRebound;
    }

};

static int test_index = register_test("Sphere Scene Test", "Wall rebound", TestWallRebound::create);
