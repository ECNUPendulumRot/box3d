
#ifndef BOX3D_UNIT_TEST_HPP
#define BOX3D_UNIT_TEST_HPP


#include "test.hpp"
#include "box3d.hpp"
#include <Eigen/Dense>

class UnitTestBase;


struct UnitTestEntry
{
    const char* name;

    TestCreateFcn* create_fcn;
};


int register_unit_test(const char* name, TestCreateFcn* fcn);

class UnitTestBase: public TestBase {

protected:

    b3BlockAllocator m_allocator;

public:

    UnitTestBase() = default;

    ~UnitTestBase() override = default;

    int get_shape_count() const override {
        return 0;
    }

    b3Shape* get_shape_list() const override {
        return nullptr;
    }

    void step() override {
        ;
    }
};

#define MAX_TEST 256

extern UnitTestEntry g_unit_test_entries[MAX_TEST];
extern int g_unit_test_count;

#endif //BOX3D_UNIT_TEST_HPP
