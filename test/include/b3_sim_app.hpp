
#ifndef BOX3D_B3_SIM_APP_HPP
#define BOX3D_B3_SIM_APP_HPP

#include "box3d.hpp"

#include <filesystem>
#include <vector>

#include <CLI/CLI.hpp>

#include "b3_test.hpp"
#include "b3_gui_viewer.hpp"

#define MAX_TEST 256

namespace box3d {

    class b3World;

}


class b3SimApp {

    std::vector<std::string> m_mesh_paths;

    std::vector<std::string> m_fixture_paths;

    CLI::App m_app;

    b3GUIViewer m_viewer;

    TestEntry m_test_series[MAX_TEST];

    TestBase* m_test;

    int m_test_count;

public:

    b3SimApp();

    ~b3SimApp();

    void load_scene(const std::string& scene_str);

    int register_test(const char* name, TestCreateFcn* fcn);

    int launch(bool gui);

private:

};

#endif //BOX3D_B3_SIM_APP_HPP
