#include "scene_test.hpp"

class TestAffineNewtonsCradle :public SceneTestBase {

public:

    TestAffineNewtonsCradle() {
        //m_world->set_solver_type(AFFINE_SOLVER);
        m_world->set_gravity(b3Vector3r(0, 0, 0));
        int num_of_spheres = 2;
        // create a dynamic body
        b3Transformr pose, velocity;

        b3BodyDef body_def;
        // body_def.m_k = 2;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(0.5);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;


        b3Vector3r p(0, 0, 0.5);
        b3Vector3r q(0, 0, 0);
        b3Vector3r v(0, 5, 0);
        b3Vector3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);

        real x = 2;

        for (int32 i = 0; i < num_of_spheres - 1; i++) {
            p = {0, x + i, 0.5};
            v = {0, 0, 0};
            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);
            m_world->create_body(body_def)->create_fixture(fixture_def);
        }

    }

    static TestBase* create() {
        return new TestAffineNewtonsCradle;
    }

};

static int test_index = register_test("Affine Scene Test", "Sphere collide", TestAffineNewtonsCradle::create);