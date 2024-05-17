
#include "test.hpp"

class TestPlanet : public Test {

    b3Body* m_planet;
    b3Body* m_second_plane;

public:

    TestPlanet() {
        b3BodyDef body_def;
        b3Vec3r v(0, 0, 0);
        b3Vec3r w(0, 0, 20);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_static_body;

        m_planet = m_world->create_body(body_def);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(2);

        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_density = 0;
        fixture_def.m_friction = 1;
        fixture_def.m_restitution = 0;

        m_planet->create_fixture(fixture_def);

        b3Vec3r p(3, 0, 0);
        b3Vec3r q(0, 0, 0);
        w.set(0, 0, 0);
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_dynamic_body;

        m_second_plane = m_world->create_body(body_def);

        sphere_shape.set_as_sphere(0.4);

        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_density = 1;

        m_second_plane->create_fixture(fixture_def);
    }

    virtual void step(Settings& settings) override {
        Test::step(settings);

        b3Vec3r dir = m_second_plane->get_position() - m_planet->get_position();
        dir.normalized();
        m_world->set_gravity(-10 * dir);
    }

    static Test* create() {
        return new TestPlanet();
    }
};

static int test_index = register_test("Sphere Scene Test", "Planet Scene", TestPlanet::create);