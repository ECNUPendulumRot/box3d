#include "box3d.hpp"
#include "b3_sim_app.hpp"
#include <filesystem>

#include <CLI/CLI.hpp>

#include <igl/opengl/glfw/Viewer.h>


int main(int argc, char* argv[]) {

    namespace fs = std::filesystem;

    set_logger_level(spdlog::level::info);


    spdlog::log(spdlog::level::info, "size: {}", sizeof (CLI::App));
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


    return 0;
}
//
// Created by sherman on 11/23/23.
//
