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
 * @brief A test class to simulate a resting box on a plane in a physics world.
 */
class TestBoxRest: public Test {

public:
    /**
     * @brief Constructor for TestBoxRest.
     */
    TestBoxRest() {

        // Set the gravity for the world to -10 in the z-axis
        m_world->set_gravity(b3Vec3r(0, 0, -10));
        // Create a dynamic cube
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Create a dynamic cube
            b3Vec3r p(0, 0, 3);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(5, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body* cube = m_world->create_body(body_def);

            // Create a cube shape with defined half-size
            b3CubeShape cube_shape;
            cube_shape.set_as_box(1.0, 1.0, 1.0);

            // Define fixture properties for the box
            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            // Create the fixture and attach it to the body
            cube->create_fixture(box_fd);
        }

//        {
//            b3BodyDef body_def;
//            body_def.m_type = b3BodyType::b3_dynamic_body;
//
//            b3Vec3r p(0, 0, 3);
//            b3Vec3r q(0, 0, 0);
//            b3Vec3r v(0, 0, 0);
//            b3Vec3r w(0, 0, 0);
//
//            body_def.set_init_pose(p, q);
//            body_def.set_init_velocity(v, w);
//
//            b3Body* cube = m_world->create_body(body_def);
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
//            cube->create_fixture(box_fd);
//        }

        ////////////////////////////////////////////////////

        // Create a static ground plane
        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        // Initial position and orientation of the ground
        b3Vec3r p(0, 0, 2);
        b3Vec3r q(0, 0, 0);
        ground_bd.set_init_pose(p, q);

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
     * @brief Factory method to create an instance of TestBoxRest.
     * @return A pointer to the created TestBoxRest instance.
     */
    static Test* create() {
        return new TestBoxRest;
    }

};
// Register the test with the test index
static int test_index = register_test("Cube Scene Test", "Cube Rest", TestBoxRest::create);
