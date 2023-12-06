
#ifndef BOX3D_B3_SIM_APP_HPP
#define BOX3D_B3_SIM_APP_HPP

#include "box3d.hpp"

#include <filesystem>
#include <vector>

#include <CLI/CLI.hpp>

#include "b3_test.hpp"

#define MAX_TEST 256

namespace box3d {

    class b3World;

}


class b3SimApp {

    std::filesystem::path m_body_def_dir;

    std::filesystem::path m_mesh_dir;

    std::filesystem::path m_scene_dir;

    std::vector<std::string> m_mesh_paths;

    std::vector<std::string> m_fixture_paths;

    box3d::b3World* m_world;

    CLI::App m_app;

    box3d::b3GUIViewer m_viewer;

    TestEntry m_test_series[MAX_TEST];

    TestBase* m_test;

    int m_test_count;

public:

    b3SimApp();

    ~b3SimApp();

    inline box3d::b3World* get_world() {
        return m_world;
    }

    void load_scene(const std::string& scene_str);

    int register_test(const char* name, TestCreateFcn* fcn);

private:

    void parse_scene(const std::string& scene_str);

    void create_object(const nlohmann::json& object);

};

#endif //BOX3D_B3_SIM_APP_HPP
