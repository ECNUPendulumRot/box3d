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
 * @brief A test class to simulate the propagation of cubes in a physics world.
 */
class TestComplexShape: public Test {

    // Number of boxes to propagate
    static constexpr int32 box_count = 8;

    // Half-size of each box
    const real box_hf_size = 1.0f;

public:

    /**
     * @brief Constructor for TestCubePropagation.
     */
    TestComplexShape() {

        // Set the gravity for the world to zero
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // Create the first dynamic cube with an initial velocity
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(0, -4, box_hf_size);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 2, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body *ball = m_world->create_body(body_def);

            // Create a cube shape with defined half-size
            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(box_hf_size);

            // Define fixture properties for the box
            b3FixtureDef box_fd;
            box_fd.m_shape = &sphere_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            // Create the fixture and attach it to the body
            ball->create_fixture(box_fd);
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(0, 0, box_hf_size);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body *cube = m_world->create_body(body_def);

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
        }

        // Create the rest of the dynamic cubes with zero initial velocity
        for (int32 i = 0; i < box_count; i++) {

            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(pow(-1,i) * (box_hf_size * (1.5-0.5*(i%4/2))), box_hf_size*2*(i/2), box_hf_size);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body *cube = m_world->create_body(body_def);

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
        }

        ////////////////////////////////////////////////////

//        b3BodyDef ground_bd;
//        ground_bd.m_type = b3BodyType::b3_static_body;
//
//        b3Body* ground_body = m_world->create_body(ground_bd);
//
//        b3PlaneShape ground_shape;
//        ground_shape.set_as_plane(20, 20);
//
//        b3FixtureDef ground_fd;
//        ground_fd.m_shape = &ground_shape;
//        ground_fd.m_friction = 0.3;
//        ground_fd.m_restitution = 1.0;
//        ground_fd.m_density = 0.0;
//
//
//        ground_body->create_fixture(ground_fd);

    }

    /**
     * @brief Factory method to create an instance of TestCubePropagation.
     * @return A pointer to the created TestCubePropagation instance.
     */
    static Test* create() {
        return new TestComplexShape;
    }

};
// Register the test with the test index
static int test_index = register_test("Cube Scene Test", "Complex Shape", TestComplexShape::create);
