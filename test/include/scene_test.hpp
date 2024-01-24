
#ifndef BOX3D_SCENE_TEST_HPP
#define BOX3D_SCENE_TEST_HPP


#include "box3d.hpp"
#include "test.hpp"


struct SceneTestEntry
{
    const char* category;

    const char* name;

    TestCreateFcn* create_fcn;
};


int register_scene_test(const char* category, const char* name, TestCreateFcn* fcn);

class SceneTestBase: public TestBase {

protected:

    b3World* m_world;

public:

    SceneTestBase();

    // TODO: check the destructor.
    ~SceneTestBase() {
        m_world->clear();
        delete m_world;
    };

    void step() override;

    b3Shape* get_shape_list() const override {
        return m_world->get_shape_list();
    }

    int get_shape_count() const override {
        return m_world->get_shape_count();
    }
};

#define MAX_TEST 256

extern SceneTestEntry g_scene_test_entries[MAX_TEST];
extern int g_scene_test_count;

#endif //BOX3D_SCENE_TEST_HPP
