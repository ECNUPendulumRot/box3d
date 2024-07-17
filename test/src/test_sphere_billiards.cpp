// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "test.hpp"

class TestBilliards :public Test {

    int layer = 10;

public:

    /**
     * @brief Constructor for TestBilliards.
     */
    TestBilliards() {

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
            b3Vec3r p(0, -3, 1);
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
        return new TestBilliards;
    }
};

// Register the test with the test index
static int test_index = register_test("Sphere Scene Test", "Billiards", TestBilliards::create);