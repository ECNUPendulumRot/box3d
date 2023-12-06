
#include "b3_sim_app.hpp"


#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include "box3d.hpp"

b3SimApp::b3SimApp()
{
    namespace fs = std::filesystem;
    m_body_def_dir = fs::path(B3D_BODY_DEF_DIR);
    m_mesh_dir     = fs::path(B3D_MESH_DIR);
    m_scene_dir    = fs::path(B3D_SCENE_DIR);

    m_world = new box3d::b3World();
}


b3SimApp::~b3SimApp()
{
    delete m_test;
}


void b3SimApp::load_scene(const std::string &scene_str)
{

    // TODO: delete world in test first

    if (scene_str.empty()) {
        spdlog::warn("The scene path is empty");
        return;
    }

    namespace fs = std::filesystem;
    std::string scene_path = (m_scene_dir / fs::path(scene_str)).string();

    TestBase::parse_scene(m_test, scene_path);
}


void b3SimApp::parse_scene(const std::string &scene_str)
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


void b3SimApp::create_object(const nlohmann::json &object)
{
    namespace fs = std::filesystem;

    std::string mesh_path = object["mesh"];
    std::string body_def_path = object["body def"];

    // TODO: check whether this is useful
    // m_mesh_paths.push_back(mesh_path);
    // m_fixture_paths.push_back(body_def_path);

    box3d::b3Mesh* mesh = box3d::b3Mesh::create_mesh((m_mesh_dir / fs::path(mesh_path)));

    box3d::b3BodyDef body_def = box3d::b3BodyDef::create_body_definition(m_body_def_dir / fs::path(body_def_path));

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

