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

/**
 * @brief A test class to simulate dynamic spheres bouncing off a ground plane and a vertical wall.
 */
class TestWallRebound :public Test {

public:

    /**
     * @brief Constructor for TestWallRebound.
     * This constructor sets up the simulation environment with bouncing spheres and static ground and wall planes.
     */
    TestWallRebound() {

        m_world->set_gravity(b3Vec3r(0, 0, -10));
        int num_of_spheres = 5;
        // create a dynamic body
        b3Transr pose, velocity;

        b3BodyDef body_def;
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


        b3Vec3r p(-0.5, -0.8, 0.5);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(2, 3.2, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);

        p = { 0.5, -0.8, 0.5 };
        v = { -2, 3.2, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        m_world->create_body(body_def)->create_fixture(fixture_def);
        real x = 0;


        for (int32 i = 0; i < num_of_spheres - 2; i++) {
            p = { 0, x + i, 0.5 };
            v = { 0, 0, 0 };
            //if(i==2) v={0,-3.0f,0};
            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);
            m_world->create_body(body_def)->create_fixture(fixture_def);
        }
        // create a ground
        p = { 0, 0, 0 };
        v = { 0, 0, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(50, 50);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        ground_body->create_fixture(fixture_def);

        p = { 0, 2.5f, 0 };
        q = {3.14159 * 0.5, 0, 0};
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        ground_body = m_world->create_body(body_def);

        ground_body->create_fixture(fixture_def);
    }

    /**
     * @brief Factory method to create an instance of TestWallRebound.
     * @return A pointer to the created TestWallRebound instance.
     */
    static Test* create() {
        return new TestWallRebound;
    }

};

// Register the test with the test index
static int test_index = register_test("Sphere Scene Test", "Wall rebound", TestWallRebound::create);
