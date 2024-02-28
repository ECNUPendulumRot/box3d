
#ifndef BOX3D_GUI_VIEWER_HPP
#define BOX3D_GUI_VIEWER_HPP

#include <utility>

#include <vector>

#include "igl/Timer.h"
#include "igl/opengl/glfw/Viewer.h"

#include "gui/gui.hpp"
#include "gui/mesh_list_view.hpp"
#include "box3d.hpp"


class b3GUIViewer {

    using Viewer = igl::opengl::glfw::Viewer;
    using ImGuiPlugin = igl::opengl::glfw::imgui::ImGuiPlugin;
    using ViewerData = igl::opengl::ViewerData;

    Viewer m_viewer;

    ImGuiPlugin m_gui_plugin;

    Gui m_menu;
    MeshListView m_mesh_list;

    TestBase* m_test = nullptr;

    // The transform matrix for
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

    void redraw_mesh();

    void add_meshes();

    void clear_meshes();

    bool check_test_index();

    void add_ground();

    bool call_back_mouse_down(Viewer& viewer, int button, int modifier);

    int allocate_mesh(const int& index);

    void draw_auxiliary_shapes();
};


#endif //BOX3D_GUI_VIEWER_HPP
