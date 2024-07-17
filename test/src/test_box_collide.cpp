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
 * @brief A test class to simulate box collision in a physics world.
 */
class TestBoxCollide: public Test {

    static constexpr int32 box_count = 2; ///< Number of boxes in the simulation
    const real box_hf_size = 1.0f; ///< Half-size of the boxes

    b3Body* m_bodys[box_count]; ///< Array to store box bodies

public:

    /**
     * @brief Constructor for TestBoxCollide.
     */
    TestBoxCollide() {
        
        // Set the gravity for the world to zero
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // Create the first dynamic box
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Create the first dynamic box
            b3Vec3r p(0,  -4.0f, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 2, 0);
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
            box_fd.m_friction = 0.0;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            // Create the fixture and attach it to the body
            cube->create_fixture(box_fd);
        }

        // Create the second dynamic box
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(0,  4.0f, 0);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, -2, 0);
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
            box_fd.m_friction = 0.0;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            // Create the fixture and attach it to the body
            cube->create_fixture(box_fd);
        }

    }


    /**
     * @brief Step the simulation.
     * @param settings Simulation settings.
     */
    void step(Settings &settings) override {
        Test::step(settings);
    }

    /**
     * @brief Factory method to create an instance of TestBoxCollide.
     * @return A pointer to the created TestBoxCollide instance.
     */
    static Test* create() {
        return new TestBoxCollide;
    }

};
// Register the test with the test index
static int test_index = register_test("Cube Scene Test", "Cube Collide", TestBoxCollide::create);
