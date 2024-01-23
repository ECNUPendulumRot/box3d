
#ifndef BOX3D_B3_UNIT_TEST_HPP
#define BOX3D_B3_UNIT_TEST_HPP


#include "box3d.hpp"
#include <Eigen/Dense>

class UnitTestBase;


using UnitTestCreateFcn = UnitTestBase*();


struct UnitTestEntry
{
    const char* name;

    UnitTestCreateFcn* create_fcn;
};


int register_unit_test(const char* name, UnitTestCreateFcn* fcn);

class UnitTestBase {

protected:


public:

    UnitTestBase() = default;

    // TODO: check the destructor.
    virtual ~UnitTestBase() = default;

    virtual int get_shape_count() const {
        return 0;
    }

    virtual b3Shape* get_shape_list() const {
        return nullptr;
    }

    virtual void step() {
        ;
    }
};

#define MAX_TEST 256

extern UnitTestEntry g_unit_test_entries[MAX_TEST];
extern int g_unit_test_count;

#endif //BOX3D_B3_UNIT_TEST_HPP
