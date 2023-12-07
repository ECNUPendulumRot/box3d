
#ifndef BOX3D_B3_GUI_VIEWER_HPP
#define BOX3D_B3_GUI_VIEWER_HPP

#include <utility>

#include "spdlog/spdlog.h"
#include "igl/Timer.h"
#include "igl/opengl/glfw/Viewer.h"

#include "box3d.hpp"
#include "b3_test.hpp"


class b3GUIViewer {

    using Viewer = igl::opengl::glfw::Viewer;

    Viewer m_viewer;

    box3d::b3World* m_world;

    std::map<int, int> m_viewer_id_to_mesh_id;

public:

    void launch();

    bool set_world(box3d::b3World* world);

private:

    bool pre_draw_loop();

    void simulation_step();

    void redraw_mesh();

    void add_meshes();

};


#endif //BOX3D_B3_GUI_VIEWER_HPP
