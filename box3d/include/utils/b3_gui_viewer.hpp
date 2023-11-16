
#ifndef BOX3D_B3_GUI_VIEWER_HPP
#define BOX3D_B3_GUI_VIEWER_HPP


#include <spdlog/spdlog.h>

#include <igl/Timer.h>
#include <igl/opengl/glfw/Viewer.h>


#include "dynamics/b3_world.hpp"


namespace box3d {

    class b3GUIViewer;

}

class box3d::b3GUIViewer {

    using Viewer = igl::opengl::glfw::Viewer;

    Viewer m_viewer;

    box3d::b3World* m_world;

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
