
#include "unit_test.hpp"

#include <spdlog/spdlog.h>

class UnitTestBoxCollide: public UnitTestBase {

    b3TransformD xf_A;

    b3TransformD xf_B;

    b3Body* body_A = nullptr;

    b3Body* body_B = nullptr;

    b3CubeShape* cube_A = nullptr;

    b3CubeShape* cube_B = nullptr;

    b3Vector3d hf_A = b3Vector3d(1, 1, 1);

    b3Vector3d hf_B = b3Vector3d(1, 1, 1);


    int selected_box = -1;
    b3Body* selected_body = nullptr;

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

    void selected_object(const int& index) override {
        if(index == 0) {
            selected_body = body_A;
        } else if(index == 1) {
            selected_body = body_B;
        }
    }

    void key_pressed(Viewer& viewer, unsigned int key, int modifiers) override {
        spdlog::log(spdlog::level::info, "child key pressed");
        if(selected_body == nullptr) {
            return;
        }
        if(key == GLFW_KEY_UP) {
            b3TransformD xf = selected_body->get_pose();
            xf.set_linear(xf.linear() + b3Vector3d(-0.1, 0, 0));
            spdlog::log(spdlog::level::info, "move forward");
            selected_body->set_pose(xf);
        } else if(key == GLFW_KEY_DOWN) {
            b3TransformD xf = selected_body->get_pose();
            xf.set_linear(xf.linear() + b3Vector3d(0.1, 0, 0));
            spdlog::log(spdlog::level::info, "move backward");
            selected_body->set_pose(xf);
        } else if(key == GLFW_KEY_LEFT) {
            b3TransformD xf = selected_body->get_pose();
            xf.set_linear(xf.linear() + b3Vector3d(0, -0.1, 0));
            spdlog::log(spdlog::level::info, "move left");
            selected_body->set_pose(xf);
        } else if(key == GLFW_KEY_RIGHT) {
            b3TransformD xf = selected_body->get_pose();
            xf.set_linear(xf.linear() + b3Vector3d(0, 0.1, 0));
            spdlog::log(spdlog::level::info, "move right");
            selected_body->set_pose(xf);
        }
    }


};

int register_index = register_test("unit test", "box collide", UnitTestBoxCollide::create);