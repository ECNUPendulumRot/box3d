
#ifndef BOX3D_SCENE_TEST_HPP
#define BOX3D_SCENE_TEST_HPP


#include "box3d.hpp"
#include "test.hpp"

class SceneTestBase: public TestBase {

protected:

    b3World* m_world;

public:

    SceneTestBase(){
        m_world = new b3World;
        m_world->set_gravity(b3Vector3d(0.0, 0.0, -10.0));
    }

    ~SceneTestBase() {
        m_world->clear();
        delete m_world;
    };

    b3World* get_world() override {
        return m_world;
    }

    void step() override {
        m_world->step(1.0 / 60, 8, 8);
    }

    b3Shape* get_shape_list() const override {
        return m_world->get_shape_list();
    }

    int get_shape_count() const override {
        return m_world->get_shape_count();
    }
};


#endif //BOX3D_SCENE_TEST_HPP
