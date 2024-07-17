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
 * @brief A test class to simulate stacking boxes in a physics world.
 */
class TestBoxStack: public Test {

     // Number of boxes to stack
    static constexpr int32 box_count = 2;

    // Half-size of each box
    const real box_hf_size = 1.0f;

    // Array to hold the box bodies
    b3Body* m_bodys[box_count];

public:

    /**
     * @brief Constructor for TestBoxStack.
     */
    TestBoxStack() {

        // Set the gravity for the world to -10 in the z-axis
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        // Create dynamic boxes and stack them
        for (int32 i = 0; i < box_count; ++i) {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(0, 0, 1 + i * box_hf_size * 2.0f);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body* cube = m_world->create_body(body_def);

            // Create a cube shape with defined half-size
            b3CubeShape cube_shape;
            cube_shape.set_as_box(box_hf_size, box_hf_size, box_hf_size);

            // Define fixture properties for the box
            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            // Create the fixture and attach it to the body
            cube->create_fixture(box_fd);

            // Store the body in the array
            m_bodys[i] = cube;
        }

        ////////////////////////////////////////////////////

        // Create a static ground plane
        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        // Create the ground body in the world
        b3Body* ground_body = m_world->create_body(ground_bd);

        // Create a plane shape with defined dimensions
        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        // Define fixture properties for the ground
        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.3;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;

        // Create the fixture and attach it to the ground body
        ground_body->create_fixture(ground_fd);

    }

    /**
     * @brief Override the step function to update the simulation and log cube velocities.
     * @param settings Simulation settings.
     */
    void step(Settings &settings) override {
        Test::step(settings);
        for (int32 i = 0; i < box_count; ++i) {
            b3Body* cube = m_bodys[i];
            b3Vec3r v = cube->get_linear_velocity();
            b3Vec3r w = cube->get_angular_velocity();
            // spdlog::info("cube[{}] v = ({}, {}, {}), w = ({}, {}, {})\n", i, v.x, v.y, v.z, w.x, w.y, w.z);
        }

    }

    /**
     * @brief Factory method to create an instance of TestBoxStack.
     * @return A pointer to the created TestBoxStack instance.
     */
    static Test* create() {
        return new TestBoxStack;
    }

};
// Register the test with the test index
static int test_index = register_test("Cube Scene Test", "Cube Stack", TestBoxStack::create);
