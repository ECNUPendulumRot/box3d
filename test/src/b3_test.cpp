
#include "b3_test.hpp"


int register_test(const char *name, TestCreateFcn *fcn)
{
    int index = g_testCount;
    if (index < MAX_TEST)
    {
        g_testEntries[index] = { name, fcn };
        ++g_testCount;
        return index;
    }

    return -1;
}


TestBase::TestBase()
{
    m_world = new box3d::b3World;
}


void TestBase::simulation_step()
{

}



