
#ifndef BOX3D_GUI_VIEWER_HPP
#define BOX3D_GUI_VIEWER_HPP

#include <utility>

#include <vector>

#include "igl/Timer.h"
#include "igl/opengl/glfw/Viewer.h"

#include "gui/gui_test_menu.hpp"
#include "gui/gui_mesh_menu.hpp"
#include "gui/gui_title_bar.hpp"

#include "box3d.hpp"


class b3GUIViewer {

public:

    using Viewer = igl::opengl::glfw::Viewer;
    using ImGuiPlugin = igl::opengl::glfw::imgui::ImGuiPlugin;
    using ViewerData = igl::opengl::ViewerData;

private:

    Viewer m_viewer;

    ImGuiPlugin m_gui_plugin;

    GuiTestMenu m_gui_test_menu;
    GuiMeshMenu m_gui_mesh_menu;
    GuiTitleBar m_gui_title_bar;

    TestBase* m_test = nullptr;

    // The transform matrix
    Eigen::Matrix3d m_transform;

    int m_current_test = -1;

    // The space for mesh data in data_list of viewer
    int m_viewer_used_count = 0;

    std::vector<b3Shape*> m_shapes;

    b3Timer m_timer;

public:

    b3GUIViewer();

    void launch();

    inline void set_max_fps(double fps) {
        m_viewer.core().animation_max_fps = fps;
    }

private:

    bool pre_draw_loop();

    void draw_mesh();

    void add_meshes();

    void clear_viewer_data();

    bool check_test_index();

    void add_ground();

    int allocate_mesh_index();

    void draw_auxiliary_shapes();

    void draw_body_axis();
};


#endif //BOX3D_GUI_VIEWER_HPP
