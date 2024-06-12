
#include "test.hpp"

class TestBallGeyser : public Test {

    b3Vec3r m_init_velocity{0, 10, 0};

    int layer = 7;

public:

    TestBallGeyser() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

//        b3PlaneShape ground_shape;
//        ground_shape.set_as_plane(100, 100);
//
//        b3BodyDef ground_body_def;
//        ground_body_def.m_type = b3BodyType::b3_static_body;
//
//        b3Body* ground = m_world->create_body(ground_body_def);
//        b3FixtureDef ground_fixture_def;
//        ground_fixture_def.m_shape = &ground_shape;
//        ground_fixture_def.m_density = 0;
//        ground_fixture_def.m_friction = 0;
//        ground_fixture_def.m_restitution = 1.0;
//
//        b3Fixture* ground_fixture = ground->create_fixture(ground_fixture_def);
//
//        utils.track_body(ground, "ground");
//        utils.track_fixture(ground_fixture, "ground");

        b3Vec3r center_pos(-5, 20, 1);
        b3Vec3r left_box_pos(-20, 10, 1);
        b3Vec3r right_box_pos(10, 10, 1);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1.0);

        b3BodyDef sphere_body_def;
        sphere_body_def.m_type = b3BodyType::b3_dynamic_body;
        b3Vec3r w;
        b3Vec3r position;
        b3Vec3r rotation;

        position.set(center_pos.x, center_pos.y - 14, 1);

        sphere_body_def.set_init_pose(position, rotation);
        sphere_body_def.set_init_velocity(m_init_velocity, w);
        b3Body* init_sphere = m_world->create_body(sphere_body_def);

        b3FixtureDef sphere_fixture_def;
        sphere_fixture_def.m_shape = &sphere_shape;
        sphere_fixture_def.m_density = 1.0;
        sphere_fixture_def.m_friction = 0.0;
        sphere_fixture_def.m_restitution = 1.0;

        b3Fixture* init_sphere_fixture = init_sphere->create_fixture(sphere_fixture_def);
        utils.track_body(init_sphere, "init_velocity_sphere");
        utils.track_fixture(init_sphere_fixture, "init_velocity_sphere");

        b3Vec3r v;
        sphere_body_def.set_init_velocity(v, w);

        int index = 1;
        real x_offset = 2.0;
        real y_offset = sqrtf(3.0f);

        int SPHERE_COUNT[5] = {4, 3, 4, 3, 2};

        for (int i = 0; i < 5; i++) {
            b3Vec3r left_pos(center_pos.x - SPHERE_COUNT[i] + 1.0, center_pos.y - i * y_offset, 1);

            for (int j = 0; j < SPHERE_COUNT[i]; j++) {
                b3Vec3r pos(left_pos.x + x_offset * j, left_pos.y, 1);
                b3Vec3r rot;

                sphere_body_def.set_init_pose(pos, rot);

                b3Body* sphere = m_world->create_body(sphere_body_def);
                b3Fixture* fixture = sphere->create_fixture(sphere_fixture_def);

                utils.track_body(sphere, ("sphere_" + std::to_string(index)).c_str());
                utils.track_fixture(fixture, ("sphere_" + std::to_string(index)).c_str());

                index++;
            }
        }

        for (int i = 1; i <= layer; i++) {
            b3Vec3r left_pos(center_pos.x - layer + i, center_pos.y + i * y_offset, 1);
            for (int j = 0; j <= layer - i; j++) {
                b3Vec3r pos(left_pos.x + j * x_offset, left_pos.y, left_pos.z);
                b3Vec3r rot;

                sphere_body_def.set_init_pose(pos, rot);

                b3Body* sphere = m_world->create_body(sphere_body_def);
                b3Fixture* fixture = sphere->create_fixture(sphere_fixture_def);

                utils.track_body(sphere, ("sphere_" + std::to_string(index)).c_str());
                utils.track_fixture(fixture, ("sphere_" + std::to_string(index)).c_str());

                index++;
            }
        }

        real hx = center_pos.x - left_box_pos.x - 4;
        real hy = center_pos.y - left_box_pos.y + y_offset - 1.0f;
        b3CubeShape cube_shape;
        cube_shape.set_as_box(hx, hy, 1);

        b3BodyDef cube_body_def;
        cube_body_def.m_type = b3BodyType::b3_static_body;

        rotation.set_zero();
        cube_body_def.set_init_pose(left_box_pos, rotation);

        b3Body* left_box_body = m_world->create_body(cube_body_def);

        b3FixtureDef left_box_fixture_def;
        left_box_fixture_def.m_shape = &cube_shape;
        left_box_fixture_def.m_density = 0;
        left_box_fixture_def.m_friction = 0;
        left_box_fixture_def.m_restitution = 1.0;

        b3Fixture* left_box_fixture = left_box_body->create_fixture(left_box_fixture_def);

        utils.track_body(left_box_body, "left_box");
        utils.track_fixture(left_box_fixture, "left_box");

        hx = right_box_pos.x - center_pos.x - 4;
        hy = center_pos.y - right_box_pos.y + y_offset - 1.0f;

        cube_shape.set_as_box(hx, hy, 1);

        b3BodyDef right_box_body_def;
        right_box_body_def.m_type = b3BodyType::b3_static_body;
        right_box_body_def.set_init_pose(right_box_pos, rotation);

        b3Body* right_box_body = m_world->create_body(right_box_body_def);

        b3FixtureDef right_box_fixture_def;
        right_box_fixture_def.m_shape = &cube_shape;
        right_box_fixture_def.m_density = 0;
        right_box_fixture_def.m_friction = 0;
        right_box_fixture_def.m_restitution = 1.0;

        b3Fixture* right_box_fixture = right_box_body->create_fixture(right_box_fixture_def);

        utils.track_body(right_box_body, "right_box");
        utils.track_fixture(right_box_fixture, "right_box");
    }

    static Test* create() {
        return new TestBallGeyser();
    }
};


static int test_index = register_test("Sphere Scene Test", "Ball Geyser", TestBallGeyser::create);