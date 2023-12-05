
#include "b3_sim_app.hpp"


#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include "box3d.hpp"

box3d::b3SimApp::b3SimApp()
{
    namespace fs = std::filesystem;
    m_body_def_dir = fs::path(B3D_BODY_DEF_DIR);
    m_mesh_dir     = fs::path(B3D_MESH_DIR);
    m_scene_dir    = fs::path(B3D_SCENE_DIR);

    m_world = new box3d::b3World();
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
    std::ifstream scene_file(scene_str);

    nlohmann::json scene_json = nlohmann::json::parse(scene_file);

    nlohmann::json& object_list = scene_json["objects"];

    Eigen::Vector3d gravity;
    from_json(scene_json["gravity"], gravity);

    m_world->set_gravity(b3Vector3d(gravity));

    for (auto& object : object_list) {
        create_object(object);
    }
}


void box3d::b3SimApp::create_object(const nlohmann::json &object)
{
    namespace fs = std::filesystem;

    std::string mesh_path = object["mesh"];
    std::string body_def_path = object["body def"];

    // TODO: check whether this is useful
    // m_mesh_paths.push_back(mesh_path);
    // m_fixture_paths.push_back(body_def_path);

    b3Mesh* mesh = b3Mesh::create_mesh((m_mesh_dir / fs::path(mesh_path)));

    b3BodyDef body_def = b3BodyDef::create_body_definition(m_body_def_dir / fs::path(body_def_path));

    Eigen::Vector3d initial_position;
    Eigen::Vector3d initial_orientation;
    Eigen::Vector3d initial_linear_velocity;
    Eigen::Vector3d initial_angular_velocity;

    from_json(object["initial position"],initial_position);
    from_json(object["initial position"],initial_orientation);
    from_json(object["initial linear velocity"],initial_linear_velocity);
    from_json(object["initial angular velocity"],initial_angular_velocity);

    b3PoseD initial_pose(initial_position, initial_orientation);
    b3PoseD initial_velocity(initial_linear_velocity, initial_angular_velocity);

    body_def.set_initial_status(initial_pose, initial_velocity);

    auto body = m_world->create_body(body_def);
    body->set_mesh(mesh);
}
