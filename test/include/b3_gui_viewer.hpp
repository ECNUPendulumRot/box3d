
#ifndef BOX3D_B3_GUI_VIEWER_HPP
#define BOX3D_B3_GUI_VIEWER_HPP

#include <utility>

#include "spdlog/spdlog.h"
#include "igl/Timer.h"
#include "igl/opengl/glfw/Viewer.h"

#include "box3d.hpp"
#include "b3_test.hpp"


class b3ViewMeshPair {

    int m_viewer_id;

    int m_mesh_id;

    b3ViewMeshPair* m_next;

public:

    b3ViewMeshPair(int viewer_id, int mesh_id):
        m_viewer_id(viewer_id),
        m_mesh_id(mesh_id),
        m_next(nullptr) {
        ;
    }

    ~b3ViewMeshPair() {
        free(m_next);
    }

    inline int get_viewer_id() const {
        return m_viewer_id;
    }

    inline int get_mesh_id() const {
        return m_mesh_id;
    }

    inline b3ViewMeshPair* next() const {
        return m_next;
    }

    void set_next(b3ViewMeshPair* next) {
        m_next = next;
    }
};


class b3GUIViewer {

    using Viewer = igl::opengl::glfw::Viewer;

    Viewer m_viewer;

    box3d::b3World* m_world;

    b3ViewMeshPair* m_pair_list;

    b3Matrix3d m_transform;

public:

    b3GUIViewer();

    void launch();

    bool set_world(box3d::b3World* world);

    inline void set_max_fps(double fps) {
        m_viewer.core().animation_max_fps = fps;
    }

private:

    bool pre_draw_loop();

    void simulation_step();

    void redraw_mesh();

    void add_meshes();

};


#endif //BOX3D_B3_GUI_VIEWER_HPP
