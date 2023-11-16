
#include "utils/b3_gui_viewer.hpp"


void box3d::b3GUIViewer::launch()
{
    m_viewer.data().clear();

    m_viewer.core().set_rotation_type(
            igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION);
    m_viewer.core().orthographic = true;
    m_viewer.core().is_animating = true;
    m_viewer.core().lighting_factor = 0.0;
    m_viewer.core().animation_max_fps = 120.0;
    m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer&) {
        return pre_draw_loop();
    };

    m_viewer.launch(false, "Simulation");
}


bool box3d::b3GUIViewer::pre_draw_loop()
{

    simulation_step();

    redraw_mesh();

    return false;
}


void box3d::b3GUIViewer::simulation_step()
{
    if (m_world->empty())
        return;

    m_world->test_step();

}

