
#ifndef BOX3D_B3_TEST_HPP
#define BOX3D_B3_TEST_HPP

#include "box3d.hpp"

class TestBase;


using TestCreateFcn = TestBase*();


struct TestEntry
{
    const char* category;

    const char* name;

    TestCreateFcn* create_fcn;
};


int register_test(const char* category, const char* name, TestCreateFcn* fcn);

class TestBase {

protected:

    box3d::b3World* m_world;

public:

    TestBase();

    // TODO: check the destructor.
    ~TestBase() = default;

    virtual void simulation_step();

    box3d::b3World* get_world() {
        return m_world;
    }

};

#define MAX_TEST 256

extern TestEntry g_test_entries[MAX_TEST];
extern int g_test_count;

#endif //BOX3D_B3_TEST_HPP
