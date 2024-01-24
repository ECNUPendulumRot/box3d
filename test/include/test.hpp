//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP


class TestBase;
class b3Shape;


using TestCreateFcn = TestBase*();


struct TestEntry
{
    const char* category;

    const char* name;

    TestCreateFcn* create_fcn;
};

int register_test(const char* category, const char* name, TestCreateFcn* fcn);

class TestBase {

public:

    virtual ~TestBase() = default;

    virtual void step() {
    };


    virtual int get_shape_count() const {
        return 0;
    }

    virtual b3Shape* get_shape_list() const {
        return nullptr;
    }

};


#define MAX_TEST 256

extern TestEntry g_test_entries[MAX_TEST];
extern int g_test_count;


#endif //BOX3D_TEST_HPP
