
#include "b3_test.hpp"

#include <fstream>


TestBase::TestBase()
{
    m_world = new box3d::b3World;
    m_world->set_gravity(b3Vector3d(0.0, 0.0, -10.0));
}


void TestBase::simulation_step()
{
    // m_world->test_step();
    m_world->step(1 / 60, 8, 8);
}


TestEntry g_test_entries[MAX_TEST] = { {nullptr} };

int g_test_count = 0;


int register_test(const char *category, const char *name, TestCreateFcn *fcn)
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






