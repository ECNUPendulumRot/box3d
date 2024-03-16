
#include "scene_test.hpp"

#include <random>

class TestDropping: public SceneTestBase {

    b3Timer m_timer;

    int count = 0;

public:

    TestDropping() {

        m_world->set_gravity(b3Vector3r(0, 0, -10));

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(10, 10);

        b3BodyDef ground_def;
        ground_def.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_def);

        b3FixtureDef fixture_def;
        fixture_def.m_friction = 0.3;
        fixture_def.m_restitution = 0.0;
        fixture_def.m_density = 0.0;
        fixture_def.m_shape = &ground_shape;

        ground_body->create_fixture(fixture_def);


       // for (int i = 0; i < 1; i++) {

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distrib_x(-5, 5);
            std::uniform_int_distribution<> distrib_y(-5, 5);
            std::uniform_int_distribution<> distrib_z(5, 10);


            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Transformr pose, velocity;
            pose.set_linear({real(distrib_x(gen)), real(distrib_y(gen)), real(distrib_z(gen))});
            pose.set_angular({0, 0, 0});
            velocity.set_linear(b3Vector3r(0, 0, 0));
            body_def.set_initial_status(pose, velocity);

            b3Body* cube = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(1.0, 1.0, 1.0);

            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            cube->create_fixture(box_fd);
        //}
    }

//    void step() override {
//        m_world->step(1.0 / 60, 8, 8);
//    }

    static TestBase* create() {
        return new TestDropping;
    }
};

static int test_index = register_test("Scene Test", "Dropping", TestDropping::create);



