
#include "scene_test.hpp"
#include <random>


class TestCubeStack: public SceneTestBase {

public:

    TestCubeStack() {

        m_world->set_gravity(b3Vector3r(0, 0, -10));

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Transformr pose, velocity;
            pose.set_linear({0, 0, 1});
            pose.set_angular({0, 0, 0});
            velocity.set_linear(b3Vector3r(0, 0, 0));
            body_def.set_initial_status(pose, velocity);

            b3Body* cube1 = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(1.0, 1.0, 1.0);

            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            cube1->create_fixture(box_fd);
        }

//        {
//            b3BodyDef body_def;
//            body_def.m_type = b3BodyType::b3_dynamic_body;
//
//            b3Transformr pose, velocity;
//            pose.set_linear({0, 0, 2});
//            pose.set_angular({0, 0, 0.7});
//            velocity.set_linear(b3Vector3r(0, 0, 0));
//            body_def.set_initial_status(pose, velocity);
//
//            b3Body* cube1 = m_world->create_body(body_def);
//
//            b3CubeShape cube_shape;
//            cube_shape.set_as_box(1.0, 1.0, 1.0);
//
//            b3FixtureDef box_fd;
//            box_fd.m_shape = &cube_shape;
//            box_fd.m_friction = 0.3;
//            box_fd.m_restitution = 1.0;
//            box_fd.m_density = 1.0;
//
//            cube1->create_fixture(box_fd);
//        }

        ////////////////////////////////////////////////////


        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.3;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;


        ground_body->create_fixture(ground_fd);

    }

    static TestBase* create() {
        return new TestCubeStack;
    }
};

static int test_index = register_test("Cube Scene Test", "Cube Stack", TestCubeStack::create);

