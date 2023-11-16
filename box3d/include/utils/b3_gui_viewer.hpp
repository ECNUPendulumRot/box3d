
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

    using Super = igl::opengl::glfw::ViewerPlugin;
    using Viewer = igl::opengl::glfw::Viewer;

    Viewer m_viewer;

    box3d::b3World* m_world;

public:

    void launch();

    bool set_world(box3d::b3World* world) {
        m_world = world;

        if (world->empty()){
            return false;
        } else
            add_meshes();
    }

private:

    bool pre_draw_loop();

    void simulation_step();

    void redraw_mesh() {
        for (int mesh_id = 0; mesh_id < box3d::b3Mesh::num_meshes(); ++mesh_id) {
            box3d::b3Mesh* mesh = box3d::b3Mesh::mesh(mesh_id);
            m_viewer.data(mesh_id).set_mesh(mesh->vertices(), mesh->faces());
        }
    }

    void add_meshes() {
        for (int mesh_id = 0; mesh_id < box3d::b3Mesh::num_meshes(); ++mesh_id) {
            m_viewer.append_mesh(true);
            box3d::b3Mesh* mesh = box3d::b3Mesh::mesh(mesh_id);
            m_viewer.data(mesh_id).set_mesh(mesh->vertices(), mesh->faces());
        }
    }

};


#endif //BOX3D_B3_GUI_VIEWER_HPP
