
#include "test.hpp"
#include "settings.hpp"

#include "spdlog/spdlog.h"

Test::Test() : m_point_count(0) {
    b3Vec3r gravity(0.0, 0.0, -10.0);
    m_world = new b3World(gravity);

    m_world->set_debug_draw(&g_debug_draw);
    m_world->set_contact_listener(this);
}


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

    m_point_count = 0;

    m_world->step(time_step, 8, 32);

    m_world->debug_draw();

    g_debug_draw.flush();

    if (settings.m_show_contact_points) {
        bool is_red = true;
        for (int i = 0; i < m_point_count; i++) {
            ContactPoint* cp = m_points + i;
            b3Vec3r p = cp->position;

            if (is_red) {
                g_debug_draw.draw_point(p, 10.0f, b3Color(1.0f, 0.f, 0.f));
            } else {
                g_debug_draw.draw_point(p, 10.0f, b3Color(0.0f, 1.f, 0.f));
            }
            is_red = !is_red;
        }
    }
}


void Test::pre_solve(b3Contact *contact, const b3Manifold* old_manifold)
{
//    b3Manifold* manifold = contact->get_manifold();
//
//    for (int i = 0; i < manifold->m_point_count && m_point_count < k_max_contact_points; i++) {
//        b3ManifoldPoint& point = manifold->m_points[i];
//
//        ContactPoint* cp = m_points + m_point_count;
//        cp->position = point.m_local_point;
//
//        m_point_count++;
//    }

    b3PersistentManifold* persistentManifold = contact->get_persistent_manifold();

    if (persistentManifold == nullptr) {
        return;
    }

    for (int i = 0; i < persistentManifold->get_contact_point_count() && m_point_count < k_max_contact_points; i++) {
        b3PersistentManifoldPoint& point = persistentManifold->get_cached_point(i);

        ContactPoint* cp = m_points + m_point_count;
        cp->position = point.m_position_world_on_A;

        // spdlog::info("fixtureA point: ({}, {}, {})", cp->position.x, cp->position.y, cp->position.z);

        m_point_count++;

        cp++;
        cp->position = point.m_position_world_on_B;

        // spdlog::info("fixtureB point: ({}, {}, {})", cp->position.x, cp->position.y, cp->position.z);

        m_point_count++;
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