
#ifndef BOX3D_SCENE_TEST_HPP
#define BOX3D_SCENE_TEST_HPP


#include "box3d.hpp"
#include "test.hpp"

class SceneTestBase: public TestBase {

protected:

    b3World* m_world;

    bool stop = false;

public:

    SceneTestBase(){
        m_world = new b3World;
        m_world->set_gravity(b3Vector3r(0.0, 0.0, -10.0));
    }

    ~SceneTestBase() override {
        m_world->clear();
        delete m_world;
    };

    b3World* get_world() override {
        return m_world;
    }

    void step() override {
        if (stop == true)
            return;
        m_world->step(1.0 / 60, 8, 8);
    }

    b3Shape* get_shape_list() const override {
        return m_world->get_shape_list();
    }

    int get_shape_count() const override {
        return m_world->get_shape_count();
    }

    bool key_pressed(Viewer &viewer, unsigned int key, int modifiers) override {

        if (key == GLFW_KEY_SPACE) {
            // Stop the world step;
            stop = !stop;
        }
        return false;
    }

};


#endif //BOX3D_SCENE_TEST_HPP
