
#include "test.hpp"


class TestConeAABB : public Test {

public:

    TestConeAABB() {
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        b3BodyDef bodyDef;
        bodyDef.m_type = b3BodyType::b3_dynamic_body;

        b3Vec3r p(0, 0, 0.5);
        b3Vec3r q(0, b3_pi * 0.5, 0);
        b3Vec3r v;
        b3Vec3r w;
        bodyDef.set_init_pose(p, q);
        bodyDef.set_init_velocity(v, w);

        b3ConeShape shape;
        shape.set_as_cone(1, 1);

        b3FixtureDef fixtureDef;
        fixtureDef.m_shape = &shape;
        fixtureDef.m_density = 1;

        b3Body* body = m_world->create_body(bodyDef);
        body->create_fixture(fixtureDef);

        {
            b3BodyDef groundBodyDef;
            groundBodyDef.m_type = b3BodyType::b3_static_body;

            b3Body* body = m_world->create_body(groundBodyDef);

            b3FixtureDef ground_fixtureDef;

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(20, 20);

            ground_fixtureDef.m_shape = &ground_shape;
            ground_fixtureDef.m_density = 0;

            body->create_fixture(ground_fixtureDef);
        }
    }

    static Test* create() {
        return new TestConeAABB;
    }
};


static int test_index = register_test("AABB", "cone", TestConeAABB::create);