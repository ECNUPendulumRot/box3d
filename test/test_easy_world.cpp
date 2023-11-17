#include "box3d.hpp"

#include <fstream>
#include <filesystem>

#include <CLI/CLI.hpp>

#include <igl/opengl/glfw/Viewer.h>
#include <nlohmann/json.hpp>

int main(int argc, char* argv[]) {

    namespace fs = std::filesystem;

    set_logger_level(spdlog::level::info);

    CLI::App app("run simulation");

    std::string mesh_path;
    std::string fixture_path;
    app.add_option("scene_path,-s,--scene-path",
                   mesh_path,
                   "JSON file with input mesh")  -> default_val("sphere.obj");

    app.add_option("fixture_path,-f,--fixture-path",
                   fixture_path,
                     "JSON file with input mesh")  -> default_val("rigid/sphere.json");

    CLI11_PARSE(app, argc, argv);

    fs::path path(mesh_path);

    path = fs::path(B3D_MESH_DIR) / path;

    spdlog::log(spdlog::level::info, "path: {}", path.string());

    if (!fs::exists(path)) {
        spdlog::warn("The path do not exist");
        return 0;
    }


////////////////////////////////////////////////////////////////////////////////////////////

    fs::path f_p(fixture_path);
    f_p =  (fs::path(B3D_FIXTURE_DIR) / f_p);

    spdlog::log(spdlog::level::info, "path: {}", f_p.string());

    std::ifstream f(f_p.string());
    nlohmann::json fixture_f = nlohmann::json::parse(f);

    box3d::b3World world;

    world.set_gravity(b3Vector3d(0.0, 0.0, -9.8));

    box3d::b3Mesh* mesh = box3d::b3Mesh::create_mesh((fs::path(B3D_MESH_DIR) / path).string());

    box3d::b3BodyDef body_def = box3d::b3BodyDef::create_body_definition(fixture_f);

    box3d::b3Body* body = world.create_body(body_def);

    body->set_mesh(mesh);

    box3d::b3GUIViewer viewer;

    viewer.set_world(&world);

    viewer.launch();

    return 0;
}
