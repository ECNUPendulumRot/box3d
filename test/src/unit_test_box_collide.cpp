
#include "unit_test.hpp"

class UnitTestBoxCollide: public UnitTestBase {

    b3TransformD xf_A;

    b3TransformD xf_B;

    b3Body* body_A = nullptr;

    b3Body* body_B = nullptr;

    b3CubeShape* cube_A = nullptr;

    b3CubeShape* cube_B = nullptr;

    b3Vector3d hf_A = b3Vector3d(1, 1, 1);

    b3Vector3d hf_B = b3Vector3d(1, 1, 1);

public :

    UnitTestBoxCollide() {
        xf_A.set_linear(0, 0, 0);
        xf_A.set_angular(0, 0, 0);
        cube_A = new b3CubeShape();
        cube_A->set_as_box(hf_A.m_x, hf_A.m_y, hf_A.m_z);
        body_A = new b3Body();
        body_A->set_pose(xf_A);
        cube_A->set_relative_body(body_A);
        cube_A->set_block_allocator(&m_allocator);


        xf_B.set_linear(b3Vector3d(0, 3, 0));
        xf_B.set_angular(0, 0, 0);
        cube_B = new b3CubeShape();
        cube_B->set_as_box(hf_B.m_x, hf_B.m_y, hf_B.m_z);
        body_B = new b3Body();
        body_B->set_pose(xf_B);
        cube_B->set_relative_body(body_B);
        cube_B->set_block_allocator(&m_allocator);

        cube_A->set_next(cube_B);
    }

    ~UnitTestBoxCollide() override {
        delete cube_A;
        delete cube_B;
        delete body_A;
        delete body_B;
    }

    void step() override {

    }

    int get_shape_count() const override {
        return 2;
    }

    b3Shape* get_shape_list() const override {
        return cube_A;
    }

    static TestBase* create() {
        return new UnitTestBoxCollide();
    }

};

int register_index = register_unit_test("box collide", UnitTestBoxCollide::create);