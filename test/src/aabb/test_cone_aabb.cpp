
#include "test.hpp"


class TestConeAABB : public Test {

    b3Body* body;
    b3Fixture* fixture;

public:

    TestConeAABB() {
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3BodyDef bodyDef;
        bodyDef.m_type = b3BodyType::b3_dynamic_body;

        b3Vec3r p;
        b3Vec3r q(0, 0, 0);
        b3Vec3r v;
        b3Vec3r w;
        bodyDef.set_init_pose(p, q);
        bodyDef.set_init_velocity(v, w);

        b3ConeShape shape;
        shape.set_as_cone(2, 4);

        b3FixtureDef fixtureDef;
        fixtureDef.m_shape = &shape;
        fixtureDef.m_density = 1;

        body = m_world->create_body(bodyDef);
        fixture = body->create_fixture(fixtureDef);

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

    void step(Settings& settings) override {
        Test::step(settings);

        b3Transformr xf;
        xf.set(body->get_position(), body->get_quaternion());

        b3AABB aabb;

//        b3Shape* shape = fixture->get_shape();
//        shape->get_bound_aabb(&aabb, xf, 0);
//
//        b3Vec3r min = aabb.min();
//        b3Vec3r max = aabb.max();
//
//        g_debug_draw.draw_point(min, 10, b3Color(1, 0, 0));
//        g_debug_draw.draw_point(max, 10, b3Color(0, 1, 0));

        // g_debug_draw.draw_plane()
    }

    static Test* create() {
        return new TestConeAABB;
    }
};


static int test_index = register_test("AABB", "cone", TestConeAABB::create);