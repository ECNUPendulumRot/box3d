//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP

#include <spdlog/spdlog.h>

#include "box3d.hpp"
#include "draw.hpp"


struct Settings;
class Test;


class Test {

protected:

    b3World* m_world;

public:

    Test();

    virtual ~Test() = default;

    virtual void step(Settings& settings);

};


using TestCreateFcn = Test*();

struct TestEntry
{
    const char* category;

    const char* name;

    TestCreateFcn* create_fcn;
};

int register_test(const char* category, const char* name, TestCreateFcn* fcn);

#define MAX_TEST 256

extern TestEntry g_test_entries[MAX_TEST];
extern int g_test_count;


#endif //BOX3D_TEST_HPP
