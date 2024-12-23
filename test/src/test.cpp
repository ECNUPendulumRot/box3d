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
#include "settings.hpp"
#include "utils.hpp"
#include "include/gl_render_triangles.hpp"

/**
 * @brief Base class for tests in the simulation.
 * This class provides common functionality for running and stepping through the simulation.
 */
Test::Test()
{
    b3Vec3r gravity(0.0, 0.0, -10.0);
    m_world = new b3World(gravity);

    m_world->set_debug_draw(&g_debug_draw);
    m_world->set_contact_listener(this);

    print_once = true;
    count = 0;
}

/**
 * @brief Steps through the simulation for a single timestep.
 * @param settings Configuration settings for the simulation.
 */
void Test::step(Settings &settings) {

    float time_step = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);
    if (settings.m_pause) {
        if (settings.m_single_step) {
            settings.m_single_step = false;
        } else {
            time_step = 0.0f;
        }
    }
    // set up flags
    uint32 flags = 0;
    flags += settings.m_draw_shapes * b3Draw::e_shape_bit;
    flags += settings.m_draw_frame_only * b3Draw::e_frame_only_bit;

    g_debug_draw.set_flags(flags);

    // before step, the point count should set to zero
    // because the point count is used to store the contact points
    m_point_count = 0;

    m_world->set_allow_sleeping(settings.m_enable_sleep);
    m_world->set_continuous_physics(settings.m_enable_continuous_physics);
    m_world->step(time_step, settings.m_velocity_iteration, 8);

    m_world->debug_draw();

    if (settings.m_generate_json) {
        utils.record_frame();
    }

    if (settings.m_output_bodies_info) {
        utils.save_body_info();
    }

    g_debug_draw.flush();
    if (settings.m_draw_contact_points) {
        for (int32 i = 0; i < m_point_count; ++i) {
            ContactPoint* cp = m_points + i;
            b3Vec3r p1 = cp->position;
            b3Vec3r p2 = p1 + 0.1f * cp->normal;
            g_debug_draw.draw_point(p1, 10.0f, b3Color(1.0f, 0.0f, 0.0f));
        }
    }

    count++;
    print_first_not_symmetry();

     //spdlog::info("====frame {}=====", count);
}

/**
 * @brief Prints a message if the system is not symmetric.
 */
void Test::print_first_not_symmetry()
{

//    b3Body** bodies = utils.get_bodies();
//    int index = 2;
//    // layer
//    for (int i = 2; i < 10; i++) {
//        for (int j = 0; j < i / 2; j++) {
//            b3Body* left_body = bodies[index + j];
//            b3Body* right_body = bodies[index + i - j - 1];
//
//            b3Vec3r left_position = left_body->get_position();
//            b3Vec3r right_position = right_body->get_position();
//
//            real center_x = left_position.x + right_position.x;
//            if (center_x > 0.01 && print_once) {
//                spdlog::info("======================not symmetric at frame {}==================", count);
//                print_once = false;
//            }
//        }
//        index += i;
//    }
}

/**
 * @brief Called before the physics solver processes a contact.
 * @param contact The contact information.
 * @param old_manifold The previous contact manifold.
 */
void Test::pre_solve(b3Contact *contact, const b3Manifold *old_manifold)
{

    b3Manifold* manifold = contact->get_manifold();
    if (manifold->m_point_count == 0) {
        return;
    }

    b3Fixture* fixture_a = contact->get_fixture_a();
    b3Fixture* fixture_b = contact->get_fixture_b();

    b3WorldManifold world_manifold;
    contact->get_world_manifold(&world_manifold);

    for (int32 i = 0; i < manifold->m_point_count && m_point_count < k_max_contact_points; i++) {
        ContactPoint* cp = m_points + m_point_count;
        cp->fixtureA = fixture_a;
        cp->fixtureB = fixture_b;
        cp->position = world_manifold.points[i];
        cp->normal = world_manifold.normal;
        ++m_point_count;
    }
}

/**
 * @brief Array of test entries for test registration.
 */
TestEntry g_test_entries[MAX_TEST] = { {nullptr} };
int g_test_count = 0;

/**
 * @brief Registers a test with the given category, name, and creation function.
 * @param category The category of the test.
 * @param name The name of the test.
 * @param fcn The function to create the test.
 * @return The index of the registered test, or -1 if registration fails.
 */
int register_test(const char* category, const char* name, TestCreateFcn* fcn)
{
    int index = g_test_count;
    if (index < MAX_TEST)
    {
        g_test_entries[index] = { category, name, fcn };
        ++g_test_count;
        return index;
    }

    return -1;// Registration failed
}