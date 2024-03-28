
#include "scene_test.hpp"

class TestVSCubeRotation : public SceneTestBase {

public:
    TestVSCubeRotation() {
        m_world->set_gravity(b3Vector3r(0, 0, -10));
        m_world->set_solver_type(VELOCITY_SOLVER);

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vector3r p(0, 0, 10);
            b3Vector3r q(2, 1, 0);
            b3Vector3r v(0, 0, 0);
            b3Vector3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* cube1 = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(1, 1, 1);

            b3FixtureDef cube_fd;
            cube_fd.m_restitution = 1;
            cube_fd.m_friction = 0.4;
            cube_fd.m_density = 1.0;
            cube_fd.m_shape = &cube_shape;

            cube1->create_fixture(cube_fd);
        }

        {
            b3BodyDef ground_def;
            ground_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(ground_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            b3FixtureDef ground_fd;
            ground_fd.m_restitution = 0;
            ground_fd.m_friction = 0.4;
            ground_fd.m_density = 0.0;
            ground_fd.m_shape = &ground_shape;

            ground_body->create_fixture(ground_fd);
        }
    }

    static TestBase* create() {
        return new TestVSCubeRotation;
    }
};

static int test_index = register_test("Velocity Solver Test", "Cube Rotation", TestVSCubeRotation::create);
