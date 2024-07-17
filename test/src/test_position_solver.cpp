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
 * @brief A test class to simulate the position solver with spheres and boxes in a physics world.
 */
class TestPositionSolver: public Test {

    // Constants for the number of spheres and boxes, and their half sizes
    static constexpr int32 sphere_count = 2;
    static constexpr int32 box_count = 1;
    const real sphere_hf = 1.0f;
    const real box_hf = 1.0f;

    // Constants for the number of spheres and boxes, and their half sizes
    b3Body* m_spheres[sphere_count];
    b3Body* m_boxes[box_count];


public:

    /**
     * @brief Constructor for TestPositionSolver.
     */
    TestPositionSolver() {

        // Set the gravity for the world
        m_world->set_gravity(b3Vec3r(0, 0, -10));


//        for (int32 i = 0; i < sphere_count; ++i) {
//            b3BodyDef body_def;
//            body_def.m_type = b3BodyType::b3_dynamic_body;
//
//            b3Vec3r p(0, 0, 1 + i * sphere_hf * 2.0f * 0.9f);
//            b3Vec3r q(0, 0, 0);
//            b3Vec3r v(0, 0, 0);
//            b3Vec3r w(0, 0, 0);
//
//            body_def.set_init_pose(p, q);
//            body_def.set_init_velocity(v, w);
//
//            b3Body* sphere = m_world->create_body(body_def);
//
//            b3SphereShape sphere_shape;
//            sphere_shape.set_as_sphere(sphere_hf);
//
//            b3FixtureDef box_fd;
//            box_fd.m_shape = &sphere_shape;
//            box_fd.m_friction = 0.3;
//            box_fd.m_restitution = 1.0;
//            box_fd.m_density = 1.0;
//
//            sphere->create_fixture(box_fd);
//            m_spheres[i] = sphere;
//        }
//
        {

            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vec3r p(0, 0, sphere_hf);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 0, 0);
            b3Vec3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* sphere = m_world->create_body(body_def);

            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(sphere_hf);

            b3FixtureDef box_fd;
            box_fd.m_shape = &sphere_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            sphere->create_fixture(box_fd);
        }

//        for (int32 i = 0; i < box_count; ++i) {
//            b3BodyDef body_def;
//            body_def.m_type = b3BodyType::b3_dynamic_body;
//
//            b3Vec3r p(0, 5, 1 + i * box_hf * 2.0f);
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
//            cube_shape.set_as_box(box_hf, box_hf, box_hf);
//
//            b3FixtureDef box_fd;
//            box_fd.m_shape = &cube_shape;
//            box_fd.m_friction = 0.3;
//            box_fd.m_restitution = 1.0;
//            box_fd.m_density = 1.0;
//
//            cube->create_fixture(box_fd);
//            m_boxes[i] = cube;
//        }

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

        // Create the fixture and attach it to the ground body
        ground_body->create_fixture(ground_fd);

    }

    /**
     * @brief Override the step function to include custom logic for each simulation step.
     * @param settings The settings for the simulation step.
     */
    void step(Settings &settings) override {
        Test::step(settings);
//        for (int32 i = 0; i < sphere_count; ++i) {
//            b3Body* cube = m_spheres[i];
//            b3Vec3r v = cube->get_linear_velocity();
//            b3Vec3r w = cube->get_angular_velocity();
//            // spdlog::info("cube[{}] v = ({}, {}, {}), w = ({}, {}, {})\n", i, v.x, v.y, v.z, w.x, w.y, w.z);
//        }

    }

    /**
     * @brief Factory method to create an instance of TestPositionSolver.
     * @return A pointer to the created TestPositionSolver instance.
     */
    static Test* create() {
        return new TestPositionSolver;
    }

};
// Register the test with the test index
static int test_index = register_test("Functional Test", "Position Solver", TestPositionSolver::create);
