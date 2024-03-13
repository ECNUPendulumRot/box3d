
#ifndef BOX3D_GUI_VIEWER_HPP
#define BOX3D_GUI_VIEWER_HPP

#include <utility>

#include <vector>

#include "igl/Timer.h"
#include "igl/opengl/glfw/Viewer.h"

#include "gui/test_menu.hpp"
#include "gui/mesh_menu.hpp"
#include "gui/title_bar.hpp"

#include "box3d.hpp"


class b3GUIViewer {

    using Viewer = igl::opengl::glfw::Viewer;
    using ImGuiPlugin = igl::opengl::glfw::imgui::ImGuiPlugin;
    using ViewerData = igl::opengl::ViewerData;

    Viewer m_viewer;

    ImGuiPlugin m_gui_plugin;

    TestMenu m_test_list;
    MeshMenu m_mesh_list;
    TitleBar m_title_bar;

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

    bool call_back_mouse_down(Viewer& viewer, int button, int modifier);

    int allocate_mesh();

    void draw_auxiliary_shapes();

};


#endif //BOX3D_GUI_VIEWER_HPP
