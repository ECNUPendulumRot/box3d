
#include "scene_test.hpp"

#include <fstream>


SceneTestBase::SceneTestBase()
{
    m_world = new b3World;
    m_world->set_gravity(b3Vector3d(0.0, 0.0, -10.0));
}


void SceneTestBase::step()
{
    // m_world->test_step();
    m_world->step(1.0 / 60, 8, 8);
}


SceneTestEntry g_scene_test_entries[MAX_TEST] = {{nullptr} };

int g_scene_test_count = 0;


int register_scene_test(const char *category, const char *name, TestCreateFcn *fcn)
{
    int index = g_scene_test_count;
    if (index < MAX_TEST)
    {
        g_scene_test_entries[index] = {category, name, fcn };
        ++g_scene_test_count;
        return index;
    }
    return -1;
}






