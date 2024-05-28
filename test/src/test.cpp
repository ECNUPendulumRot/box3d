
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

    // before step, the point count should set to zero
    // because the point count is used to store the contact points
    m_point_count = 0;

    m_world->set_allow_sleeping(settings.m_enable_sleep);

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