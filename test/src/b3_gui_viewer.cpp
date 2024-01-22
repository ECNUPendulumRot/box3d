
#include "../include/b3_gui_viewer.hpp"
#include "utils/b3_log.hpp"

#include "b3_test.hpp"

static inline bool compare_tests(const TestEntry& a, const TestEntry& b)
{
    int result = strcmp(a.category, b.category);
    if (result == 0)
    {
        result = strcmp(a.name, b.name);
    }

    return result < 0;
}


static void sort_tests()
{
    std::sort(g_test_entries, g_test_entries + g_test_count, compare_tests);
}


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

    sort_tests();

    m_viewer.core().camera_eye = Eigen::Vector3f(0, 0, 10.0f);
}


bool b3GUIViewer::set_world(b3World *world) {

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
    // m_viewer.core().orthographic = true;
    m_viewer.core().is_animating = true;
    // m_viewer.core().lighting_factor = 0.0;
    m_viewer.core().animation_max_fps = 60.0;

    m_viewer.plugins.push_back(&m_plugin);
    m_plugin.widgets.push_back(&m_menu);

    m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer&) {
        return pre_draw_loop();
    };

    m_viewer.launch(false, "Simulation");
}


bool b3GUIViewer::pre_draw_loop()
{
    // make sure that two variables are initialized to -1
    if (m_menu.m_selected_test != m_current_test) {
        m_current_test = m_menu.m_selected_test;

        if (m_test != nullptr)
            delete m_test;
        m_test = g_test_entries[m_menu.m_selected_test].create_fcn();
        m_world = m_test->get_world();
        clear_meshes();
        add_meshes();
    }

    if (m_test != nullptr)
        m_test->simulation_step();

    redraw_mesh();

    return false;
}


void b3GUIViewer::simulation_step()
{
    if (m_world == nullptr || m_world->empty())
        return;

    m_world->test_step();

}


void b3GUIViewer::add_meshes() {

    int shape_count = m_world->get_shape_count();

    b3Shape* shape = m_world->get_shape_list();

    while(shape) {
        int viewer_id = m_viewer.append_mesh(true); 
        m_view_id_vector.push_back(viewer_id);


        b3ViewData* view_data;
        view_data = shape->get_view_data();

        E3MapMatrixX<double, Eigen::RowMajor> vertices(view_data->m_V, view_data->m_vertex_count, 3);
        E3MapMatrixX<int, Eigen::RowMajor> faces(view_data->m_F, view_data->m_face_count, 3);

        m_viewer.data(viewer_id).set_mesh(vertices, faces);

        // TODO: use b3_alloc
        // auto* pair = new b3ViewShapePair(viewer_id, index);

        // pair->set_next(m_pair_list);

        // m_pair_list = pair;

        // index++;
        shape = shape->next();
    }

    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 0, 0), Eigen::RowVector3d(1, 0, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 1, 0), Eigen::RowVector3d(0, 1, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1), Eigen::RowVector3d(0, 0, 1));

}

void b3GUIViewer::clear_meshes() {
    b3ViewShapePair* pair = m_pair_list;
    while(pair) {
        b3ViewShapePair* destory_pair = pair;
        pair = pair->next();
        b3_free(destory_pair);
    }

    m_pair_list = nullptr;
}


void b3GUIViewer::redraw_mesh() {

    if(m_world == nullptr) {
        return;
    }

    b3ViewShapePair* pair = m_pair_list;

    b3Shape* shape = m_world->get_shape_list();

    int index = 0;

    while (shape != nullptr) {

        // int viewer_id = pair->get_viewer_id();
        // int mesh_id = pair->get_mesh_id();

        b3ViewData* view_data;
        view_data = shape->get_view_data();
        E3MapMatrixX<double, Eigen::RowMajor> vertices(view_data->m_V, view_data->m_vertex_count, 3);
        E3MapMatrixX<int, Eigen::RowMajor> faces(view_data->m_F, view_data->m_face_count, 3);

        m_viewer.data(m_view_id_vector[index]).set_mesh(vertices, faces);

        index++;

        // pair = pair->next();
        shape = shape->next();
    }

}


