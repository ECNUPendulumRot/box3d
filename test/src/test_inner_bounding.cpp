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
 * @brief A test class to simulate spheres within a bounding box in a physics world.
 */
class TestInnerBounding :public Test {

    // Number of spheres along one dimension
    int c_count = 4;

    // Radius of each sphere
    real radius = 0.5;

public:
    /**
     * @brief Constructor for TestInnerBounding.
     */
    TestInnerBounding() {
        // Set the gravity for the world to zero
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // Define fixture properties for spheres
        b3FixtureDef fixture_def;
        fixture_def.m_friction = 0;
        fixture_def.m_restitution = 1.0;
        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(radius);

        // Create the first dynamic sphere with an initial velocity
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity
            b3Vec3r p(c_count * radius, -5, c_count * radius);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 10, 0);
            b3Vec3r w(0, 0, 0);

            // Create the body in the world
            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            // Create the body in the world
            b3Body* b = m_world->create_body(body_def);

            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_density = 1.0;

            // Create the fixture and attach it to the body
            b3Fixture* f = b->create_fixture(fixture_def);

            // Create the fixture and attach it to the body
            utils.track_body(b, "init_velocity_sphere");
            utils.track_fixture(f, "init_velocity_sphere");

            p={c_count * radius, 5+2 * c_count * radius, c_count * radius};
            v={0,0,0};

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b = m_world->create_body(body_def);
            f = b->create_fixture(fixture_def);

            utils.track_body(b, "back_sphere");
            utils.track_fixture(f, "back_sphere");
        }

        // Create a grid of dynamic spheres with zero initial velocity
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            int index = 1;

            b3Vec3r q(0, 0, 0);
            for(int i = 0; i < c_count; i++) {
                b3Vec3r left_position(radius, radius + 2 * i * radius, radius);
                for(int j = 0; j < c_count; j++) {
                    for (int k = 0; k < c_count; k++) {
                        b3Vec3r p = left_position + 2 * j * b3Vec3r(radius, 0, 0) + 2 * k * b3Vec3r(0, 0, radius);
                        body_def.set_init_pose(p, q);
                        body_def.set_init_velocity(b3Vec3r(0, 0, 0), b3Vec3r(0, 0, 0));
                        b3Body* b = m_world->create_body(body_def);
                        b3Fixture* f = b->create_fixture(fixture_def);

                        utils.track_body(b, ("sphere_" + std::to_string(index)).c_str());
                        // Track each fixture for debugging or analysis purposes
                        utils.track_fixture(f, ("sphere_" + std::to_string(index)).c_str());
                        index++;
                    }
                }
            }
        }
        b3Vec3r p;
        b3Vec3r q;
        {
            b3BodyDef body_def;
            p = { 0, 0, -10};
            q = {0, 0, 0};
            body_def.set_init_pose(p, q);
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;
            b3Fixture* fg = ground_body->create_fixture(fixture_def);
        }

        {
            // create a ground
            b3BodyDef body_def;
            p = { 0, 0, 2 * c_count * radius + 10};
            q = {3.1415926535897, 0, 0};
            body_def.set_init_pose(p, q);
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;
            b3Fixture* fg = ground_body->create_fixture(fixture_def);
        }

        {
            // create a ground
            b3BodyDef body_def;
            p = { 0, -10, 0};
            q = {-3.1415926535897 / 2.0, 0, 0};
            body_def.set_init_pose(p, q);
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;
            b3Fixture* fg = ground_body->create_fixture(fixture_def);
        }

        {
            // create a ground
            b3BodyDef body_def;
            p = { 0, 2 * radius * c_count + 10, 0};
            q = {3.1415926535897 / 2.0, 0, 0};
            body_def.set_init_pose(p, q);
            body_def.m_type = b3BodyType::b3_static_body;
            b3Body* ground_body = m_world->create_body(body_def);

            b3PlaneShape ground_shape;
            ground_shape.set_as_plane(100, 100);

            fixture_def.m_shape = &ground_shape;
            fixture_def.m_density = 0;
            b3Fixture* fg = ground_body->create_fixture(fixture_def);
        }

//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { -10, 0, 0};
//            q = {0, 3.1415926535897 / 2.0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
//
//        {
//            // create a ground
//            b3BodyDef body_def;
//            p = { 2 * radius * c_count + 10, 0, 0};
//            q = {0, -3.1415926535897 / 2.0, 0};
//            body_def.set_init_pose(p, q);
//            body_def.m_type = b3BodyType::b3_static_body;
//            b3Body* ground_body = m_world->create_body(body_def);
//
//            b3PlaneShape ground_shape;
//            ground_shape.set_as_plane(100, 100);
//
//            fixture_def.m_shape = &ground_shape;
//            fixture_def.m_density = 0;
//            b3Fixture* fg = ground_body->create_fixture(fixture_def);
//        }
    }
    /**
     * @brief Factory method to create an instance of TestInnerBounding.
     * @return A pointer to the created TestInnerBounding instance.
     */
    static Test* create() {
        return new TestInnerBounding;
    }
};

// Register the test with the test index
static int test_index = register_test("Sphere Scene Test", "Inner Bounding", TestInnerBounding::create);
