
#include "b3_sim_app.hpp"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

box3d::b3SimApp::b3SimApp()
{
    namespace fs = std::filesystem;
    m_fixture_dir = fs::path(B3D_FIXTURE_DIR);
    m_mesh_dir    = fs::path(B3D_MESH_DIR);
    m_scene_dir   = fs::path(B3D_SCENE_DIR);

}

void box3d::b3SimApp::load_scene(const std::string &scene_str)
{
    if (scene_str.empty()) {
        spdlog::warn("The scene path is empty");
        return;
    }

    namespace fs = std::filesystem;
    std::string scene_path = (m_scene_dir / fs::path(scene_str)).string();

    parse_scene(scene_path);
}

void box3d::b3SimApp::parse_scene(const std::string &scene_str)
{
    nlohmann::json scene_json = nlohmann::json::parse(scene_str);

}
