
#include "b3_test.hpp"

#include <fstream>

namespace fs = std::filesystem;

fs::path TestBase::s_body_def_dir = fs::path(B3D_BODY_DEF_DIR);
fs::path TestBase::s_mesh_dir = std::filesystem::path(B3D_MESH_DIR);
fs::path TestBase::s_scene_dir =std::filesystem::path(B3D_SCENE_DIR);


TestBase::TestBase()
{
    m_world = new box3d::b3World;
}


void TestBase::simulation_step()
{
    m_world->test_step();
}


void TestBase::create_object(const nlohmann::json &object)
{
    namespace fs = std::filesystem;

    std::string mesh_path = object["mesh"];
    std::string body_def_path = object["body def"];

    // TODO: check whether this is useful
    // m_mesh_paths.push_back(mesh_path);
    // m_fixture_paths.push_back(body_def_path);

    box3d::b3Mesh* mesh = box3d::b3Mesh::create_mesh((s_mesh_dir / fs::path(mesh_path)));

    box3d::b3BodyDef body_def = box3d::b3BodyDef::create_body_definition(s_body_def_dir / fs::path(body_def_path));

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


void TestBase::parse_scene(TestBase* test, const std::string &scene_str)
{

    // World must be cleared before the new scene parsed
    box3d::b3World* world = test->get_world();
    world->clear();

    std::ifstream scene_file((s_scene_dir / fs::path(scene_str)).string());
    nlohmann::json scene_json = nlohmann::json::parse(scene_file);

    Eigen::Vector3d gravity;
    from_json(scene_json["gravity"], gravity);
    world->set_gravity(b3Vector3d(gravity));

    nlohmann::json& object_list = scene_json["objects"];
    for (auto& object : object_list) {
        test->create_object(object);
    }
}




