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
 * @brief A test class to simulate the interaction of a dynamic sphere with a stack of cubes and a ground plane.
 */
class TestWallBreak: public Test {

    int32 box_count = 8;
    const real box_hf_size = 0.5f;

public:

    /**
     * @brief Constructor for TestWallBreak.
     * This constructor sets up the simulation environment with a sphere and a stack of cubes arranged in a pyramid.
     */
    TestWallBreak() {

// Set gravity for the simulation world
        m_world->set_gravity(b3Vec3r(0, 0, -10));

        b3Vec3r p(0, 0, 0);   ///< Initial position
        b3Vec3r q(0, 0, 0);   ///< No initial rotation
        b3Vec3r v(0, 0, 0);   ///< No initial velocity
        b3Vec3r w(0, 0, 0);   ///< No initial angular velocity

        b3BodyDef body_def;
        body_def.m_type = b3BodyType::b3_dynamic_body;

        // Define a dynamic sphere with an initial velocity
        p = {20, 0, 7};      ///< Initial position of the sphere
        q = {0, 0, 0};       ///< No rotation
        v = {-40, 0, 0};     ///< Velocity directed towards the stack of cubes
        w = {0, 0, 0};       ///< No angular velocity
        body_def.set_init_pose(p, q);
        body_def.set_init_velocity(v, w);

        b3SphereShape sphere_shape;
        sphere_shape.set_as_sphere(1.6);
        b3FixtureDef sphere_fd;
        sphere_fd.m_shape = &sphere_shape;
        sphere_fd.m_friction = 0.3;
        sphere_fd.m_restitution = 0.3;
        sphere_fd.m_density = 1.0;

        b3Body *sphere = m_world->create_body(body_def);
        b3Fixture* fs = sphere->create_fixture(sphere_fd);
        utils.track_fixture(fs, "hitting_sphere");

        int32 scale = 1;
        real x_pos = 0;
        x_pos += box_hf_size;
        int32 index = 0;
        for (int32 k = 0; k < 3; k++) {
            for (int32 i = 0; i < box_count; i++) {
                for (int32 j = 0; j < box_count / 2; j++) {

                    b3BodyDef body_def;
                    body_def.m_type = b3BodyType::b3_dynamic_body;

                    p = {x_pos, (scale * box_hf_size + j * scale * box_hf_size * 2.0f), box_hf_size * scale + i * box_hf_size * scale * 2.0f};
                    q = {0, 0, 0};
                    v = {0, 0, 0};
                    w = {0, 0, 0};
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);

                    b3CubeShape cube_shape;
                    cube_shape.set_as_box(box_hf_size, scale * box_hf_size, scale * box_hf_size);
                    b3FixtureDef box_fd;
                    box_fd.m_shape = &cube_shape;
                    box_fd.m_friction = 0.3;
                    box_fd.m_restitution = 0.3;
                    box_fd.m_density = 1.0;

                    b3Body *cube_r = m_world->create_body(body_def);
                    b3Fixture* fr = cube_r->create_fixture(box_fd);
                    utils.track_fixture(fr, ("cube" + std::to_string(index++)).c_str());

                    p = {x_pos, -(scale * box_hf_size + j * scale * box_hf_size * 2.0f), box_hf_size * scale + i * box_hf_size * scale * 2.0f};
                    body_def.set_init_pose(p, q);
                    body_def.set_init_velocity(v, w);
                    b3Body *cube_l = m_world->create_body(body_def);
                    b3Fixture* fl = cube_l->create_fixture(box_fd);
                    utils.track_fixture(fl, ("cube" + std::to_string(index++)).c_str());
                }
            }
            scale *= 2;
            box_count /= 2;
            x_pos += 2 * box_hf_size;
        }


        ////////////////////////////////////////////////////

        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(100, 100);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.3;
        ground_fd.m_restitution = 0.2;
        ground_fd.m_density = 0.0;

        b3Fixture* fg = ground_body->create_fixture(ground_fd);
        utils.track_fixture(fg, "ground");
    }

    /**
     * @brief Factory method to create an instance of TestWallBreak.
     * @return A pointer to the created TestWallBreak instance.
     */
    static Test* create() {
        return new TestWallBreak;
    }

};

// Register the test with the test index
static int test_index = register_test("Cube Scene Test", "Wall Break", TestWallBreak::create);
