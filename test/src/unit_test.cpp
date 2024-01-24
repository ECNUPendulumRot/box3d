
#include "unit_test.hpp"


UnitTestEntry g_unit_test_entries[MAX_TEST] = {{nullptr} };

int g_unit_test_count = 0;


int register_unit_test(const char *name, TestCreateFcn *fcn)
{
    int index = g_unit_test_count;
    if (index < MAX_TEST)
    {
        g_unit_test_entries[index] = { name, fcn };
        ++g_unit_test_count;
        return index;
    }
    return -1;
}
