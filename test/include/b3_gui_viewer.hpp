
#ifndef BOX3D_B3_GUI_VIEWER_HPP
#define BOX3D_B3_GUI_VIEWER_HPP

#include <utility>

#include "spdlog/spdlog.h"

#include "igl/Timer.h"
#include "igl/opengl/glfw/Viewer.h"

#include "b3_category_menu.hpp"

#include "box3d.hpp"

class b3ViewShapePair {

    int m_viewer_id;

    int m_shape_id;

    b3ViewShapePair* m_next;

public:

    b3ViewShapePair(int viewer_id, int mesh_id):
            m_viewer_id(viewer_id),
            m_shape_id(mesh_id),
            m_next(nullptr) {
        ;
    }

    ~b3ViewShapePair() {
        free(m_next);
    }

    inline int get_viewer_id() const {
        return m_viewer_id;
    }

    inline int get_mesh_id() const {
        return m_shape_id;
    }

    inline b3ViewShapePair* next() const {
        return m_next;
    }

    void set_next(b3ViewShapePair* next) {
        m_next = next;
    }
};


class b3GUIViewer {

    using Viewer = igl::opengl::glfw::Viewer;
    using ImGuiMenu = igl::opengl::glfw::imgui::ImGuiMenu;
    using ImGuiPlugin = igl::opengl::glfw::imgui::ImGuiPlugin;

    Viewer m_viewer;

    ImGuiPlugin m_plugin;

    CategoryMenu m_menu;

    box3d::b3World* m_world;

    TestBase* m_test = nullptr;

    b3ViewShapePair* m_pair_list;

    E3Matrix3d m_transform;

    int m_current_test = -1;

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
