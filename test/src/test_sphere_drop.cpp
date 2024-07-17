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
#include <random>

/**
 * @brief A test class to simulate a single sphere dropping from a height onto a static ground plane.
 */
class TestSphereDrop: public Test {

public:

    /**
     * @brief Constructor for TestSphereDrop.
     */
    TestSphereDrop() {
        
        // Set gravity in the simulation world (downward)
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        // Create a dynamic sphere body
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, and velocity for the sphere
            b3Vec3r p(0, 0, 10);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the sphere body in the simulation world
            b3Body* sphere = m_world->create_body(body_def);

            // Define the sphere shape
            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(2.0);

            // Define the fixture properties for the sphere
            b3FixtureDef sphere_fd;
            sphere_fd.m_shape = &sphere_shape;
            sphere_fd.m_friction = 0.3;
            sphere_fd.m_restitution = 1.0;
            sphere_fd.m_density = 1.0;

            // Attach the fixture to the sphere body
            sphere->create_fixture(sphere_fd);
        }

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

    /**
     * @brief Factory method to create an instance of TestSphereDrop.
     * @return A pointer to the created TestSphereDrop instance.
     */
    static Test* create() {
        return new TestSphereDrop;
    }

};
// Register the test with the test index
static int test_index = register_test("Cube Scene Test", "Sphere Drop", TestSphereDrop::create);
