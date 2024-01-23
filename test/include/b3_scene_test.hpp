
#ifndef BOX3D_B3_SCENE_TEST_HPP
#define BOX3D_B3_SCENE_TEST_HPP

#include "box3d.hpp"

class SceneTestBase;


using SceneTestCreateFcn = SceneTestBase*();


struct SceneTestEntry
{
    const char* category;

    const char* name;

    SceneTestCreateFcn* create_fcn;
};


int register_scene_test(const char* category, const char* name, SceneTestCreateFcn* fcn);

class SceneTestBase {

protected:

    b3World* m_world;

public:

    SceneTestBase();

    // TODO: check the destructor.
    ~SceneTestBase() = default;

    virtual void step();

    b3World* get_world() {
        return m_world;
    }

};

#define MAX_TEST 256

extern SceneTestEntry g_scene_test_entries[MAX_TEST];
extern int g_scene_test_count;

#endif //BOX3D_B3_SCENE_TEST_HPP
