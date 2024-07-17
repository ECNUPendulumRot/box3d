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
 * @brief A test class to simulate a Newton's cradle-like scene with multiple dynamic spheres and a static ground.
 */
class SphereCradleTest : public Test {

    // Number of dynamic spheres in the simulation
    int sphere_count = 3;

public:

    /**
     * @brief Constructor for SphereCradleTest.
     */
    SphereCradleTest() {

        // Set the gravity for the world to zero (no gravity)
        m_world->set_gravity(b3Vec3r(0, 0, 0));

        // Create a static ground plane
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_static_body;

            // Define the shape of the plane
            b3PlaneShape plane_shape;
            plane_shape.set_as_plane(100, 100);

            // Define the fixture properties for the plane
            b3FixtureDef fixture_def;
            fixture_def.m_shape = &plane_shape;
            fixture_def.m_density = 0.0;
            fixture_def.m_restitution = 1.0;
            fixture_def.m_friction = 0.0;

            // Create the ground body in the world and attach the fixture to it
            b3Body* ground = m_world->create_body(body_def);
            b3Fixture* fg = ground->create_fixture(fixture_def);

            // Track the ground body and fixture for debugging or analysis
            utils.track_body(ground, "ground");
            utils.track_fixture(fg, "ground");
        }
        
        // Create dynamic spheres
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            // Initial position, orientation, linear velocity, and angular velocity for the first sphere
            b3Vec3r p(0, -3, 1);
            b3Vec3r q(0, 0, 0);
            b3Vec3r v(0, 5, 0);
            b3Vec3r w(0, 0, 0);

            // create a sphere shape
            b3SphereShape sphere_shape;
            sphere_shape.set_as_sphere(1.0);

            // create a fixture definition
            b3FixtureDef fixture_def;
            fixture_def.m_shape = &sphere_shape;
            fixture_def.m_friction = 0;
            fixture_def.m_restitution = 1.0;
            fixture_def.m_density = 1.0;

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* b = m_world->create_body(body_def);
            utils.track_body(b, "init_velocity_sphere");

            b3Fixture* f = b->create_fixture(fixture_def);
            utils.track_fixture(f, "init_velocity_sphere");

            // Set the velocity to zero for the rest of the spheres
            v.set_zero();
            body_def.set_init_velocity(v, w);

            // Create the remaining spheres
            for(int i = 1; i <= sphere_count; i++) {
                b3Vec3r position(0, 1 + 2 * i, 1);
                body_def.set_init_pose(position, q);

                b3Body* bi  = m_world->create_body(body_def);
                b3Fixture* fi = bi->create_fixture(fixture_def);

                utils.track_body(bi, ("sphere_" + std::to_string(i)).c_str());
                utils.track_fixture(fi, ("sphere_" + std::to_string(i)).c_str());

            }
        }
    }

    /**
     * @brief Factory method to create an instance of SphereCradleTest.
     * @return A pointer to the created SphereCradleTest instance.
     */
    static Test* create() {
        return new SphereCradleTest();
    }
};
// Register the test with the test index
static int test_index = register_test("Sphere Scene Test", "Cradle", SphereCradleTest::create);