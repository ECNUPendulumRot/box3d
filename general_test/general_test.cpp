//
// Created by sherman on 23-11-15.
//
#include <iostream>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>


int main(int argc, char* argv[]) {

    Eigen::MatrixXd m = Eigen::Matrix3d();

    std::cout << sizeof(Eigen::MatrixXd) << std::endl;
    std::cout << sizeof(Eigen::MatrixXi) << std::endl;
    std::cout << sizeof(Eigen::MatrixXf) << std::endl;


    std::cout << sizeof(decltype(m)::Scalar) * m.size() << std::endl;

    igl::opengl::glfw::Viewer viewer;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    viewer.launch();

    return 0;
}
