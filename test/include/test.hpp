//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP

#include "igl/opengl/glfw/Viewer.h"
#include <spdlog/spdlog.h>

class TestBase;
class b3Shape;
class b3AuxiliaryShape;
class b3World;

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

    using Viewer = igl::opengl::glfw::Viewer;

public:

    virtual ~TestBase() = default;

    virtual void step() {
    };

    virtual b3World* get_world() {
        return nullptr;
    }


    virtual int get_shape_count() const {
        return 0;
    }

    virtual b3Shape* get_shape_list() const {
        return nullptr;
    }

    virtual int get_auxiliary_shape_count() const {
        return 0;
    }

    virtual b3AuxiliaryShape* get_auxiliary_shape_list() const {
        return nullptr;
    }

    virtual void selected_object(const int& index) {
        ;
    }

    virtual bool key_pressed(Viewer& viewer, unsigned int key, int modifiers) {
        return false;
    }

};


#define MAX_TEST 256

extern TestEntry g_test_entries[MAX_TEST];
extern int g_test_count;


#endif //BOX3D_TEST_HPP
