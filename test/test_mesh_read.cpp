#include "box3d.hpp"

#include <filesystem>

#include <CLI/CLI.hpp>

#include <igl/opengl/glfw/Viewer.h>


int main(int argc, char* argv[]) {

    namespace fs = std::filesystem;

    set_logger_level(spdlog::level::info);

    CLI::App app("run simulation");

    std::string mesh_path;

    app.add_option("scene_path,-i,-s,--scene-path",
                   mesh_path,
                   "JSON file with input mesh")  -> default_val("sphere.obj");

    CLI11_PARSE(app, argc, argv);

    fs::path path(mesh_path);

    path = fs::path(B3D_MESH_DIR) / path;

    spdlog::log(spdlog::level::info, "path: {}", path.string());

    if (!fs::exists(path)) {
        spdlog::warn("The path do not exist");
        return 0;
    }

    spdlog::log(spdlog::level::info, "path: {}", path.string());

    auto* body = new box3d::b3RigidBody( (fs::path(B3D_MESH_DIR) / path).string());

    igl::opengl::glfw::Viewer viewer;

    viewer.data().set_mesh(body->mesh()->vertices(), body->mesh()->faces());

    viewer.data().set_face_based(true);

    viewer.launch();

    return 0;
}
