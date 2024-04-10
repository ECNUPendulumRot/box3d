
#include "scene_test.hpp"
#include "auxiliary_shape.hpp"

class TestAffineEmpty: public SceneTestBase {

    b3Timer m_timer;

    int count = 0;

public:

    TestAffineEmpty() {
        //m_world->set_solver_type(AFFINE_SOLVER);
        m_world->set_gravity(b3Vector3r(0, 0, -10));

        ////////////////////////////////////////////////////

        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.0;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;


        ground_body->create_fixture(ground_fd);
    }

    void step() override {
        m_world->step(1.0 / 60, 8, 8);

    }

    static TestBase* create() {
        return new TestAffineEmpty;
    }

};

static int test_index = register_test("Affine Scene Test", "empty", TestAffineEmpty::create);



