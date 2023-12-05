
#include "utils/b3_gui_viewer.hpp"
#include "utils/b3_log.hpp"




bool box3d::b3GUIViewer::set_world(box3d::b3World *world) {
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


void box3d::b3GUIViewer::launch()
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


void box3d::b3GUIViewer::add_meshes() {
    for (int mesh_id = 0; mesh_id < box3d::b3Mesh::num_meshes(); ++mesh_id) {
        printf("mesh_id: %d\n", mesh_id);
        int viewer_id = m_viewer.append_mesh(true);
        box3d::b3Mesh* mesh = box3d::b3Mesh::mesh(mesh_id);
        m_viewer.data(mesh_id).set_mesh(mesh->vertices(), mesh->faces());

        m_viewer_id_to_mesh_id[viewer_id] = mesh_id;
    }
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 0, 0), Eigen::RowVector3d(1, 0, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 1, 0), Eigen::RowVector3d(0, 1, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1), Eigen::RowVector3d(0, 0, 1));

}


void box3d::b3GUIViewer::redraw_mesh() {
    static b3Matrix3d transform = [](){
        b3Matrix3d m;
        m.setIdentity();
        m.col(0).swap(m.col(2));
        m.col(1).swap(m.col(2));
        return m;
    }();

    for (int viewer_id = 0; viewer_id < m_viewer.data_list.size(); ++viewer_id) {
        box3d::b3Mesh* mesh = box3d::b3Mesh::mesh(m_viewer_id_to_mesh_id[viewer_id]);

        auto vertices = mesh->transform();

        m_viewer.data(viewer_id).set_mesh(vertices * transform.transpose(), mesh->faces());
    }
}


