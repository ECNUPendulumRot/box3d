
#ifndef BOX3D_B3_SIM_APP_HPP
#define BOX3D_B3_SIM_APP_HPP

#include "box3d.hpp"

#include <filesystem>
#include <vector>

#include <CLI/CLI.hpp>

namespace box3d {

    class b3SimApp;

    class b3World;
}


class box3d::b3SimApp {

    std::filesystem::path m_body_def_dir;

    std::filesystem::path m_mesh_dir;

    std::filesystem::path m_scene_dir;

    CLI::App app;

    std::vector<std::string> m_mesh_paths;

    std::vector<std::string> m_fixture_paths;

    box3d::b3World* m_world;

public:

    b3SimApp();

    inline b3World* get_world() {
        return m_world;
    }

    void load_scene(const std::string& scene_str);

//void parse(int argc, char* argv[]);

private:

    void parse_scene(const std::string& scene_str);

    void create_object(const nlohmann::json& object);

};

#endif //BOX3D_B3_SIM_APP_HPP
