
#include "test.hpp"
#include "settings.hpp"
#include "utils.hpp"
#include "include/gl_render_triangles.hpp"

Test::Test()
{
    b3Vec3r gravity(0.0, 0.0, -10.0);
    m_world = new b3World(gravity);

    m_world->set_debug_draw(&g_debug_draw);
    m_world->set_contact_listener(this);

    print_once = true;
    count = 0;
}


void Test::step(Settings &settings) {

    float hertz = settings.m_hertz;
    if (settings.m_pause) {
        if (settings.m_single_step) {
            settings.m_single_step = false;
        } else {
            hertz = 0.0;
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

    m_solver = b3Solver::get_solver(b3Solver::e_substep_split_island, m_world, &m_world->m_block_allocator);

    m_world->step(hertz, settings.m_velocity_iteration, 8, settings.m_main_iteration,m_solver);

    m_solver->destroy();

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

    spdlog::info("====frame {}=====", count);
}


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


TestEntry g_test_entries[MAX_TEST] = { {nullptr} };
int g_test_count = 0;

int register_test(const char* category, const char* name, TestCreateFcn* fcn)
{
    int index = g_test_count;
    if (index < MAX_TEST)
    {
        g_test_entries[index] = { category, name, fcn };
        ++g_test_count;
        return index;
    }

    return -1;
}