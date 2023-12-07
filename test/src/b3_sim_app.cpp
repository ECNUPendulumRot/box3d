
#include "b3_sim_app.hpp"

#include <spdlog/spdlog.h>


b3SimApp::b3SimApp()
{
    namespace fs = std::filesystem;

    m_test = nullptr;
    m_test_count = 0;

}


b3SimApp::~b3SimApp()
{
    delete m_test;
}


void b3SimApp::load_scene(const std::string &scene_str)
{
    if (scene_str.empty()) {
        spdlog::warn("The scene path is empty");
        return;
    }

    // TODO: clarify test from cpp and scene from json
    m_test = new TestBase;

    TestBase::parse_scene(m_test, scene_str);
}


int b3SimApp::register_test(const char *name, TestCreateFcn *fcn)
{
    int index = m_test_count;
    if (index < MAX_TEST)
    {
        m_test_series[index] = { name, fcn };
        ++m_test_count;
        return index;
    }

    return -1;
}


int b3SimApp::launch(bool gui) {

    if (gui) {
        m_viewer.set_world(m_test->get_world());
        m_viewer.launch();
    }
    else {
        while (true) {
            m_test->simulation_step();
        }
    }

    return 0;
}

