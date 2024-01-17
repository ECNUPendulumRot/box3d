//
// Created by sherman on 23-11-15.
//
#include <iostream>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <type_traits>

uint8_t sign_of_distance(double val) {
    uint64_t bits = *reinterpret_cast<uint64_t*>(&val);
    return static_cast<uint8_t>(bits >> 63);
}

int main(int argc, char* argv[]) {

    Eigen::MatrixXd m = Eigen::Matrix3d();

    int a = 1;
    int b = 16;
    std::cout << (a xor b) << std::endl;

   // auto tu = signbit_to_uint8_double(t);

    //printf("%x", tu);


//    igl::opengl::glfw::Viewer viewer;
//
//    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
//    viewer.plugins.push_back(&plugin);
//
//    igl::opengl::glfw::imgui::ImGuiMenu menu;
//    plugin.widgets.push_back(&menu);
//
//    viewer.launch();

    return 0;
}
