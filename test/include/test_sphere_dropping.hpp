
#ifndef BOX3D_TEST_SPHERE_DROPPING_HPP
#define BOX3D_TEST_SPHERE_DROPPING_HPP

#include "b3_test.hpp"

class TestSphereDropping: public TestBase {



public:

    TestSphereDropping() {



    }

    static TestBase* create() {
        return new TestSphereDropping;
    }
};

#endif //BOX3D_TEST_SPHERE_DROPPING_HPP
