//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP

class TestBase;
class b3Shape;


using TestCreateFcn = TestBase*();


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


#endif //BOX3D_TEST_HPP
