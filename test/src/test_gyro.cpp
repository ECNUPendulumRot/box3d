
#include "test.hpp"

class TestGyro : public Test {

public:

    TestGyro() {

        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3BodyDef ground_body_def;
        ground_body_def.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(100, 100);

        b3FixtureDef ground_fixture_def;
        ground_fixture_def.m_shape = &ground_shape;
        ground_fixture_def.m_restitution = 0;
        ground_fixture_def.m_density = 0;
        ground_fixture_def.m_friction = 1.0;

        ground_body->create_fixture(ground_fixture_def);


        b3BodyDef gyro_body_def;
        gyro_body_def.m_type = b3BodyType::b3_dynamic_body;
        b3Vec3r p(0, 0, b3_sqrt(3.0f));
        b3Vec3r q;
        b3Vec3r v;
        b3Vec3r w(0, 0, 200);

        // Rx(45)Ry(45) = [ sqrt(2) / 2, 0, sqrt(2) / 2
        //                  0.5, sqrt(2) / 2, -0.5
        //                  -0.5 sqrt(2) / 2, 0.5]
        real sqrt_2 = b3_sqrt(2.0);
        real theta = 1.0 / cos((sqrt_2 + 0.5 - 1) * 0.5);
        b3Vec3r n(sqrt_2 * 0.5 + 0.5, sqrt_2 * 0.5 + 0.5, 0.5);
        n = n * 0.5 / sin(theta);

        q = theta * n;

        gyro_body_def.set_init_pose(p, q);
        gyro_body_def.set_init_velocity(v, w);

        b3Mat33r R = gyro_body_def.m_init_q.rotation_matrix();
        spdlog::info("R: {} {} {} | {} {} {} | {} {} {}", R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));

        b3Vec3r point = R * b3Vec3r(1, -1, -1);
        gyro_body_def.m_init_p.z = -point.z;

        b3Body* gyro_body = m_world->create_body(gyro_body_def);

        b3CubeShape gyro_shape;
        gyro_shape.set_as_box(1, 1, 1);

        b3FixtureDef gyro_fixture_def;
        gyro_fixture_def.m_shape = &gyro_shape;
        gyro_fixture_def.m_density = 1.0;
        gyro_fixture_def.m_restitution = 0;
        gyro_fixture_def.m_friction = 0.4;

        gyro_body->create_fixture(gyro_fixture_def);
    }


    static Test* create() {
        return new TestGyro;
    }

};


static int test_index = register_test("Cube Scene Test", "Gyro", TestGyro::create);