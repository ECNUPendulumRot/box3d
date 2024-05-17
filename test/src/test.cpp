
#include "test.hpp"
#include "settings.hpp"

Test::Test() {
    b3Vec3r gravity(0.0, 0.0, -10.0);
    m_world = new b3World(gravity);

    m_world->set_debug_draw(&g_debug_draw);
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

    m_world->step(time_step, 100, 32);

    m_world->debug_draw();

    g_debug_draw.flush();
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