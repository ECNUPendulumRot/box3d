
#ifndef BOX3D_B3_SIM_APP_HPP
#define BOX3D_B3_SIM_APP_HPP

#include <filesystem>
#include <vector>

#include <CLI/CLI.hpp>

namespace box3d {

    class b3SimApp;

}


class box3d::b3SimApp {

    std::filesystem::path m_fixture_dir;

    std::filesystem::path m_mesh_dir;

    std::filesystem::path m_scene_dir;

    CLI::App app;

    std::vector<std::string> m_mesh_paths;

    std::vector<std::string> m_fixture_paths;

    int32 m_mesh_count;

    int32 m_fixture_count;

public:

    b3SimApp();

    void load_scene(const std::string& scene_str);

//void parse(int argc, char* argv[]);

private:

    void parse_scene(const std::string& scene_str);

};

#endif //BOX3D_B3_SIM_APP_HPP
