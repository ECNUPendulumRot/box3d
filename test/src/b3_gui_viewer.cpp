
#include "../include/b3_gui_viewer.hpp"
#include "utils/b3_log.hpp"


b3GUIViewer::b3GUIViewer()
{
    m_world = nullptr;
    m_pair_list = nullptr;

    // Set up the transformation matrix
    // 0 1 0
    // 0 0 1
    // 1 0 0
    m_transform.setIdentity();
    m_transform.col(0).swap(m_transform.col(2));
    m_transform.col(1).swap(m_transform.col(2));
}


bool b3GUIViewer::set_world(box3d::b3World *world) {

    m_world = world;

    if (world->empty()){
        return false;
    } else {
        printf("world not empty\n");
        m_viewer.data().clear();
        add_meshes();
    }
    return true;
}


void b3GUIViewer::launch()
{
    m_viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION);
    //m_viewer.core().orthographic = true;
    m_viewer.core().is_animating = true;
//    m_viewer.core().lighting_factor = 0.0;
    m_viewer.core().animation_max_fps = 60.0;

    m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer&) {
        return pre_draw_loop();
    };

    m_viewer.launch(false, "Simulation");
}


bool b3GUIViewer::pre_draw_loop()
{

    simulation_step();

    redraw_mesh();

    return false;
}


void b3GUIViewer::simulation_step()
{
    if (m_world->empty())
        return;

    m_world->test_step();

}


void b3GUIViewer::add_meshes() {

    int mesh_count = m_world->get_shape_count();

    for (int i = 0; i < mesh_count; ++i) {
        int viewer_id = m_viewer.append_mesh(true);

        box3d::b3Shape* shape = m_world->get_shape(i);
        box3d::b3ViewData view_data;
        shape->get_view_data(&view_data);
        m_viewer.data(viewer_id).set_mesh(view_data.vertexes(), view_data.faces());

        // TODO: use b3_alloc
        auto* pair = new b3ViewShapePair(viewer_id, i);

        pair->set_next(m_pair_list);

        m_pair_list = pair;
    }

    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 0, 0), Eigen::RowVector3d(1, 0, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 1, 0), Eigen::RowVector3d(0, 1, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1), Eigen::RowVector3d(0, 0, 1));

}


void b3GUIViewer::redraw_mesh() {

    b3ViewShapePair* pair = m_pair_list;

    while (pair != nullptr) {

        int viewer_id = pair->get_viewer_id();
        int mesh_id = pair->get_mesh_id();

        box3d::b3Shape* shape = m_world->get_shape(mesh_id);

        box3d::b3ViewData view_data;
        shape->get_view_data(&view_data);
        m_viewer.data(viewer_id).set_mesh(view_data.vertexes() * m_transform.transpose(), view_data.faces());

        pair = pair->next();
    }

}



