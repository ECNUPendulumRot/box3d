
#ifndef BOX3D_UNIT_TEST_HPP
#define BOX3D_UNIT_TEST_HPP


#include "test.hpp"
#include "box3d.hpp"


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


#endif //BOX3D_UNIT_TEST_HPP
