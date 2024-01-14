

#include "spdlog/spdlog.h"
#include "igl/opengl/glfw/Viewer.h"

#include "box3d.hpp"

#include <iostream>

int main() {
    igl::opengl::glfw::Viewer viewer;

    box3d::b3SphereShape sphere;
    sphere.set_as_sphere(1.0);

    box3d::b3ViewData* view_data = new box3d::b3ViewData();

    sphere.get_view_data(view_data);

    spdlog::info("sphere has {} vertices.", view_data->m_V.rows());
    spdlog::info("sphere has {} faces.", view_data->m_F.rows());


    viewer.data().set_mesh(view_data->m_V, view_data->m_F);


    viewer.launch();
}