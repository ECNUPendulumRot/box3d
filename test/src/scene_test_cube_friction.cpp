
#include "scene_test.hpp"

class TestCubeFriction : public SceneTestBase {

public:
    TestCubeFriction() {
      m_world->set_gravity(b3Vector3r(0, 0, -10));

      {
        b3BodyDef bode_def;
        bode_def.m_type = b3BodyType::b3_dynamic_body;

        b3Transformr pose, velocity;
        pose.set_linear({0, -4, 1});
        pose.set_angular({0, 0, 0});
        velocity.set_linear({0, 5, 0});
        bode_def.set_initial_status(pose, velocity);

        b3Body* cube1 = m_world->create_body(bode_def);

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
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_friction = 0.4;
        ground_fd.m_density = 0.0;
        ground_fd.m_shape = &ground_shape;

        ground_body->create_fixture(ground_fd);
      }
    }

    static TestBase* create() {
      return new TestCubeFriction;
    }
};

static int test_index = register_test("Cube Scene Test", "Cube Friction", TestCubeFriction::create);