#include "test.hpp"

class TestRhombusSymmetric :public Test {

    int layer = 3;

public:

    /**
     * @brief Constructor for TestBilliards.
     */
    TestRhombusSymmetric() {

        // Set the gravity for the world to zero (no gravity)
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // Define the fixture properties for the spheres
        b3FixtureDef fixture_def;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;

        // Define the shape of the spheres
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1);

        // Create an initial sphere with a specified initial velocity
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(0,  - 3, 1);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 5, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body* b = m_world->create_body(body_def);

            // Set the shape and density for the fixture
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1.0;

            // Create the fixture and attach it to the body
            b3Fixture* f = b->create_fixture(fixture_def);

            // Track the body and fixture for debugging or analysis
            utils.track_body(b, "init_velocity_sphere");
            utils.track_fixture(f, "init_velocity_sphere");

            p = b3Vec3r(0, sqrtf(3.0f) * (2.0f * layer - 2.0f) + 9.0001, 1);
            v = b3Vec3r(0, -5, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b = m_world->create_body(body_def);
            f = b->create_fixture(fixture_def);

            utils.track_body(b, "init_velocity_sphere_reverse");
            utils.track_fixture(f, "init_velocity_sphere_reverse");
        }

        // Create multiple layers of spheres arranged in a triangular formation
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            int index = 1;

            real y_distance = sqrtf(3.0);
            b3Vec3r x_offset(-2, 0, 0);

            b3Vec3r q(0, 0, 0);
            for(int i = 0; i < layer; i++) {
                b3Vec3r left_position(i, 3 + i * y_distance, 1);
                for(int j = 0; j <= i; j++) {
                    b3Vec3r p = left_position + j * x_offset;
                    body_def.set_init_pose(p, q);

                    b3Body* b = m_world->create_body(body_def);
                    b3Fixture* f = b->create_fixture(fixture_def);

                    // Track the body and fixture for debugging or analysis
                    utils.track_body(b, ("sphere_" + std::to_string(index)).c_str());
                    utils.track_fixture(f, ("sphere_" + std::to_string(index)).c_str());
                    index++;
                }
            }
            for(int i = layer; i < 2 * layer - 1; i++) {
                b3Vec3r left_position(2.0f * layer - i - 2 , 3 + 0.9999 * i * y_distance, 1);
                for(int j = 0; j < 2 * layer - i - 1; j++) {
                    b3Vec3r p = left_position + j * x_offset;
                    body_def.set_init_pose(p, q);

                    b3Body* b = m_world->create_body(body_def);
                    b3Fixture* f = b->create_fixture(fixture_def);

                    // Track the body and fixture for debugging or analysis
                    utils.track_body(b, ("sphere_" + std::to_string(index)).c_str());
                    utils.track_fixture(f, ("sphere_" + std::to_string(index)).c_str());
                    index++;
                }
            }
        }

        // Create a static ground plane
        {
            // create a ground
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            // Set the shape and density for the fixture
            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;

            // Create the fixture and attach it to the ground body
            b3Fixture* fg = ground_body->create_fixture(fixture_def);

            // Track the ground body and fixture for debugging or analysis
            utils.track_body(ground_body, "ground");
            utils.track_fixture(fg, "ground");

        }
    }

    /**
     * @brief Factory method to create an instance of TestBilliards.
     * @return A pointer to the created TestBilliards instance.
     */
    static Test* create() {
        return new TestRhombusSymmetric;
    }
};

// Register the test with the test index
static int test_index = register_test("Sphere Scene Test", "RhombusSymmetric+", TestRhombusSymmetric::create);
