
#ifndef BOX3D_SCENE_TEST_HPP
#define BOX3D_SCENE_TEST_HPP


#include "box3d.hpp"
#include "test.hpp"


#include "auxiliary_shape.hpp"

class SceneTestBase: public TestBase {

protected:

    b3World* m_world;

    bool stop = false;

    bool step_by_step = false;

    b3AuxiliaryShape* m_local_axis = nullptr;

public:

    SceneTestBase(){
        m_world = new b3World;
        m_world->set_gravity(b3Vector3r(0.0, 0.0, -10.0));
    }

    ~SceneTestBase() override {
        m_world->clear();
        delete m_world;
        if (m_local_axis != nullptr)
            delete [] m_local_axis;
    };

    b3World* get_world() override {
        return m_world;
    }

    void step() override {
        if (stop == true) {
            return;
        }

        m_world->step(1.0 / 60, 8, 8);
    }

    b3Shape* get_shape_list() const override {
        return m_world->get_shape_list();
    }

    int32 get_shape_count() const override {
        return m_world->get_shape_count();
    }

    b3AuxiliaryShape* get_local_axis() override {

        int32 body_count = m_world->get_body_count();
        m_local_axis = new b3AuxiliaryShape[body_count];

        for (int32 i = 1; i < body_count; i++) {
            m_local_axis[i - 1].set_next(m_local_axis + i);
        }

        b3Body* body = m_world->get_body_list();
        int32 index = 0;
        while (body != nullptr) {
            b3Transformr xf(body->get_affine_q());
            const b3Vector3r p = xf.position();
            const b3Matrix3r& R = xf.rotation_matrix();

            b3Vector3r e = p + 1 * R.col(0);
            m_local_axis->add_line(p.x(), p.y(), p.z(), e.x(), e.y(), e.z());
            m_local_axis->add_color(1, 0, 0);

            e = p + 1 * R.col(1);
            m_local_axis->add_line(p.x(), p.y(), p.z(), e.x(), e.y(), e.z());
            m_local_axis->add_color(0, 1, 0);

            e = p + 1 * R.col(2);
            m_local_axis->add_line(p.x(), p.y(), p.z(), e.x(), e.y(), e.z());
            m_local_axis->add_color(0, 0, 1);

            body = body->next();
            index++;
        }

        return m_local_axis;
    }

    bool key_pressed(Viewer &viewer, unsigned int key, int modifiers) override {

        if (key == GLFW_KEY_SPACE) {
            // Stop the world step;
            stop = !stop;
        }

        if (key == GLFW_KEY_S) {
            // Step by step
            m_world->step(1.0 / 60, 8, 8);
        }

        return false;
    }

};


#endif //BOX3D_SCENE_TEST_HPP
