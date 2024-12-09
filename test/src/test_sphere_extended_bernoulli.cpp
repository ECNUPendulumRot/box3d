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
 * @brief A test class to simulate a series of spheres falling and bouncing on a static ground plane.
 */
class TestExtendedBernoulli :public Test {

    int32 sphere_count = 8;

public:

    /**
     * @brief Constructor for TestExtendedBernoulli.
     */
    TestExtendedBernoulli() {

        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // create a dynamic body
        b3Transr pose, velocity;

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // create a sphere shape
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1);

        // create a fixture definition
        b3FixtureDef fixture_def;
        fixture_def.m_shape = &sphere_shape;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        fixture_def.m_density = 1.0;

        b3Vec3r p(0, -3, 1);
        b3Vec3r q(0, 0, 0);
        b3Vec3r v(0, 5, 0);
        b3Vec3r w(0, 0, 0);

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        b3Body* b = m_world->create_body(body_def);
        utils.track_body(b, "init_velocity_sphere");

        b3Fixture* f = b->create_fixture(fixture_def);
        utils.track_fixture(f, "init_velocity_sphere");

        v = { 0, 0, 0 };

        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        p = {-sphere_count + 1.0f, 3, 1};
        for (int i = 1; i <= sphere_count; i++) {
            body_def.set_init_pose(p, q);
            b3Body* bi  = m_world->create_body(body_def);
            b3Fixture* fi = bi->create_fixture(fixture_def);

            utils.track_body(bi, ("sphere_" + std::to_string(i)).c_str());
            utils.track_fixture(fi, ("sphere_" + std::to_string(i)).c_str());
            p.x += 2;
        }

        // create a ground
        p = { 0, 0, 0 };
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);
        body_def.m_type = b3BodyType::b3_static_body;
        b3Body* ground_body = m_world->create_body(body_def);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(50, 50);

        fixture_def.m_shape = &ground_shape;
        fixture_def.m_density = 0;

        b3Fixture* fg = ground_body->create_fixture(fixture_def);

        utils.track_body(ground_body, "ground");
        utils.track_fixture(fg, "ground");
    }

    /**
     * @brief Factory method to create an instance of TestExtendedBernoulli.
     * @return A pointer to the created TestExtendedBernoulli instance.
     */
    static Test* create() {
        return new TestExtendedBernoulli;
    }

};

// Register the test with the test index
static int test_index = register_test("Sphere Scene Test", "Extended Bernoulli", TestExtendedBernoulli::create);