
#include "test.hpp"
#include "settings.hpp"

Test::Test() {
    b3Vec3r gravity(0.0, 0.0, -10.0);
    m_world = new b3World(gravity);

    m_world->set_debug_draw(&g_debug_draw);
}


void Test::step(Settings &settings) {

    // set up flags
    uint32 flags = 0;
    flags += settings.m_draw_shapes * b3Draw::e_shape_bit;
    g_debug_draw.set_flags(flags);

    m_world->step(1.0/60.0, 8, 8);

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