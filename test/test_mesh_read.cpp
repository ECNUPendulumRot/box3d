#include "box3d.hpp"

#include <filesystem>

#include <CLI/CLI.hpp>

#include <igl/opengl/glfw/Viewer.h>

box3d::b3Mesh* mesh = nullptr;

bool callback(igl::opengl::glfw::Viewer& viewer)
{
    mesh->test_addition();

    // 更新网格显示
    viewer.data().set_mesh(mesh->vertices(), mesh->faces());

    return false; // 返回false表示不退出程序
}


int main(int argc, char* argv[]) {

    namespace fs = std::filesystem;

    set_logger_level(spdlog::level::info);

    CLI::App app("run simulation");

    std::string mesh_path;

    app.add_option("scene_path,-i,-s,--scene-path",
                   mesh_path,
                   "JSON file with input mesh")  -> default_val("plane.obj");

    CLI11_PARSE(app, argc, argv);

    fs::path path(mesh_path);

    path = fs::path(B3D_MESH_DIR) / path;

    spdlog::log(spdlog::level::info, "path: {}", path.string());

    if (!fs::exists(path)) {
        spdlog::warn("The path do not exist");
        return 0;
    }

    spdlog::log(spdlog::level::info, "path: {}", path.string());

    mesh = box3d::b3Mesh::create_mesh(path.string());

    igl::opengl::glfw::Viewer viewer;

    viewer.data().set_mesh(mesh->vertices(), mesh->faces());

    viewer.data().set_face_based(true);

    viewer.callback_pre_draw = &callback;
    viewer.core().animation_max_fps = 60;
    viewer.launch();

    return 0;
}
