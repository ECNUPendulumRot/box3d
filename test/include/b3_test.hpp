
#ifndef BOX3D_B3_TEST_HPP
#define BOX3D_B3_TEST_HPP

#include "box3d.hpp"


class TestBase;



using TestCreateFcn = TestBase*();

struct TestEntry
{
    const char* name;
    TestCreateFcn* creatFcn;
};

int register_test(const char* category, const char* name, TestCreateFcn* fcn);

class TestBase {

    box3d::b3World *m_world;

public:

    TestBase();

    virtual void simulation_step();



};

#define MAX_TEST 256

extern TestEntry g_testEntries[MAX_TEST];
extern int g_testCount;

#endif //BOX3D_B3_TEST_HPP
